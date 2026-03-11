'''Low_cmd safety relay for full-body H12 control'''

import argparse
import signal
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from h12_safety_layer.utility.chunk_logger import ChunkLogger
from h12_safety_layer.utility.config import MOTOR_COUNT, load_config
from h12_safety_layer.core.safety_checks import (
    CommandValidationError,
    EStopTriggered,
    assert_state_within_estop_limits,
    make_estop_cmd_like,
    sanitize_and_clip_command,
)


LOG_SAMPLES_PER_CHUNK = 1000
LOG_WRITE_HZ = 5.0
LOG_MAX_QUEUE_SIZE = 10000


class LowCmdSafetyRelay:
    '''Relay low_cmd with validation, clipping, estop, and async logging'''

    def __init__(self, cfg: dict[str, Any]):
        self._cfg = cfg
        self._crc = CRC()
        self._lock = threading.Lock()
        self._estop = False
        self._estop_reason = ''
        self._running = False
        self._last_cmd: LowCmd_ = LowCmdDefault()
        self._last_q_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_dq_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_tau_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)

        if self._cfg['network']['interface']:
            ChannelFactoryInitialize(self._cfg['network']['domain_id'], self._cfg['network']['interface'])
        else:
            ChannelFactoryInitialize(self._cfg['network']['domain_id'])

        self._cmd_sub = ChannelSubscriber(self._cfg['topics']['low_cmd_in'], LowCmd_)
        self._state_sub = ChannelSubscriber(self._cfg['topics']['low_state'], LowState_)
        self._cmd_pub = ChannelPublisher(self._cfg['topics']['low_cmd_out'], LowCmd_)
        self._cmd_pub.Init()

        self._logger: ChunkLogger | None = None
        if self._cfg['logging']['enabled']:
            self._logger = ChunkLogger(
                base_dir=self._cfg['logging']['base_dir'],
                chunk_prefix=self._cfg['logging']['chunk_prefix'],
                samples_per_chunk=LOG_SAMPLES_PER_CHUNK,
                write_hz=LOG_WRITE_HZ,
                max_queue_size=LOG_MAX_QUEUE_SIZE,
            )
            self._logger.start()

    def start(self) -> None:
        '''Start subscriptions and keep relay alive'''

        self._validate_mode()
        self._running = True

        self._state_sub.Init(self._on_low_state, 10)
        self._cmd_sub.Init(self._on_low_cmd, 10)
        self._log_event({'event': 'relay_started', 'mode': self._cfg['mode']})

        while self._running:
            time.sleep(0.2)

    def stop(self) -> None:
        '''Stop relay and flush logger'''

        self._running = False
        self._cmd_sub.Close()
        self._state_sub.Close()
        self._cmd_pub.Close()
        self._log_event({'event': 'relay_stopped'})
        if self._logger:
            self._logger.stop()

    def _validate_mode(self) -> None:
        mode = self._cfg['mode'].strip().lower()
        if mode == 'full_body_mode':
            return
        if mode == 'split_mode':
            raise NotImplementedError('mode split_mode is reserved for future dual-low_state support')
        raise ValueError(f'unknown mode: {self._cfg["mode"]}')

    def _on_low_cmd(self, msg: LowCmd_) -> None:
        recv_ns = time.time_ns()
        with self._lock:
            self._last_cmd = msg
            self._log_event({'event': 'rx_low_cmd', 'recv_ns': recv_ns, 'motor_count': MOTOR_COUNT})

            if self._estop:
                out = make_estop_cmd_like(msg)
                self._publish_checked(out, source='estop_passthrough')
                return

            try:
                out, clipped = sanitize_and_clip_command(msg, self._cfg['limits'])
            except CommandValidationError as exc:
                self._trigger_estop(f'command validation failed: {exc}')
                out = make_estop_cmd_like(msg)
                self._publish_checked(out, source='invalid_cmd_estop')
                return

            self._publish_checked(out, source='relay', clipped=clipped)

    def _on_low_state(self, msg: LowState_) -> None:
        time_stamp = time.time()

        mode = np.asarray([int(msg.motor_state[i].mode) for i in range(MOTOR_COUNT)], dtype=np.uint8)
        q = np.asarray([float(msg.motor_state[i].q) for i in range(MOTOR_COUNT)], dtype=np.float32)
        dq = np.asarray([float(msg.motor_state[i].dq) for i in range(MOTOR_COUNT)], dtype=np.float32)
        ddq = np.asarray([float(msg.motor_state[i].ddq) for i in range(MOTOR_COUNT)], dtype=np.float32)
        tau_est = np.asarray([float(msg.motor_state[i].tau_est) for i in range(MOTOR_COUNT)], dtype=np.float32)
        temperature = np.asarray([msg.motor_state[i].temperature for i in range(MOTOR_COUNT)], dtype=np.int16)
        vol = np.asarray([float(msg.motor_state[i].vol) for i in range(MOTOR_COUNT)], dtype=np.float32)
        sensor = np.asarray([msg.motor_state[i].sensor for i in range(MOTOR_COUNT)], dtype=np.uint32)
        motorstate = np.asarray([int(msg.motor_state[i].motorstate) for i in range(MOTOR_COUNT)], dtype=np.uint32)
        reserve = np.asarray([msg.motor_state[i].reserve for i in range(MOTOR_COUNT)], dtype=np.uint32)

        imu_quaternion = np.asarray(msg.imu_state.quaternion, dtype=np.float32)
        imu_gyroscope = np.asarray(msg.imu_state.gyroscope, dtype=np.float32)
        imu_accelerometer = np.asarray(msg.imu_state.accelerometer, dtype=np.float32)
        imu_rpy = np.asarray(msg.imu_state.rpy, dtype=np.float32)
        imu_temperature = np.int16(msg.imu_state.temperature)

        self._log_sample(
            {
                'time_stamp': time_stamp,
                'mode': mode,
                'q': q,
                'dq': dq,
                'ddq': ddq,
                'tau_est': tau_est,
                'temperature': temperature,
                'vol': vol,
                'sensor': sensor,
                'motorstate': motorstate,
                'reserve': reserve,
                'imu_quaternion': imu_quaternion,
                'imu_gyroscope': imu_gyroscope,
                'imu_accelerometer': imu_accelerometer,
                'imu_rpy': imu_rpy,
                'imu_temperature': imu_temperature,
                'q_cmd': self._last_q_cmd.copy(),
                'dq_cmd': self._last_dq_cmd.copy(),
                'tau_cmd': self._last_tau_cmd.copy(),
            }
        )

        with self._lock:
            if self._estop:
                return
            try:
                assert_state_within_estop_limits(msg, self._cfg['limits'])
            except EStopTriggered as exc:
                self._trigger_estop(str(exc))
                out = make_estop_cmd_like(self._last_cmd)
                self._publish_checked(out, source='state_estop')

    def _publish_checked(self, msg: LowCmd_, source: str, clipped: int = 0) -> None:
        msg.crc = self._crc.Crc(msg)
        self._cmd_pub.Write(msg)

        self._last_q_cmd = np.asarray([float(msg.motor_cmd[i].q) for i in range(MOTOR_COUNT)], dtype=np.float32)
        self._last_dq_cmd = np.asarray([float(msg.motor_cmd[i].dq) for i in range(MOTOR_COUNT)], dtype=np.float32)
        self._last_tau_cmd = np.asarray([float(msg.motor_cmd[i].tau) for i in range(MOTOR_COUNT)], dtype=np.float32)

        self._log_event({'event': 'tx_low_cmd', 'source': source, 'clipped_count': clipped, 'estop': self._estop})

    def _trigger_estop(self, reason: str) -> None:
        if self._estop:
            return
        self._estop = True
        self._estop_reason = reason
        self._log_event({'event': 'estop_triggered', 'reason': reason})

    def _log_event(self, data: dict[str, Any]) -> None:
        _ = data

    def _log_sample(self, sample: dict[str, Any]) -> None:
        if self._logger:
            self._logger.log_sample(sample)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='h12 low_cmd safety relay')
    parser.add_argument(
        '--config',
        type=str,
        default=str(Path(__file__).resolve().parents[1] / 'config' / 'default_safety.yaml'),
        help='path to yaml safety config',
    )
    return parser


def main() -> None:
    '''Start the full-body low_cmd safety relay'''

    args = _build_arg_parser().parse_args()
    cfg = load_config(args.config)
    relay = LowCmdSafetyRelay(cfg)

    stop_event = threading.Event()

    def _handle_signal(signum: int, _frame: Any) -> None:
        _ = signum
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    thread = threading.Thread(target=relay.start, name='relay_main', daemon=True)
    thread.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.2)
    finally:
        relay.stop()


if __name__ == '__main__':
    main()
