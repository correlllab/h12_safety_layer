'''Safety layer for full-body H12 control'''

import time
import json
import threading
from typing import Any

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from h12_safety_layer.core.chunk_logger import ChunkLogger
from h12_safety_layer.core.config import MOTOR_COUNT
from h12_safety_layer.core.safety_checks import (
    CmdValidationError,
    EStopTriggered,
    check_estop_limits,
    make_estop_cmd,
    clip_low_cmd,
)


LOG_SAMPLES_PER_CHUNK = 1000
LOG_WRITE_HZ = 5.0
LOG_MAX_QUEUE_SIZE = 10000


class SafetyLayer:
    '''Relay low_cmd with validation, clipping, estop, and async logging'''
    def __init__(self, config: dict[str, Any]):
        self._config = config
        self._lock = threading.Lock()
        self._estop = False
        self._estop_reason = ''
        self._running = False
        # init DDS network interface
        if self._config['network']['interface']:
            ChannelFactoryInitialize(self._config['network']['domain_id'],
                                     self._config['network']['interface'])
        else:
            ChannelFactoryInitialize(self._config['network']['domain_id'])
        # store last command
        self._last_cmd: LowCmd_ = LowCmdDefault()
        self._last_q_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_dq_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_tau_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_kp_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_kd_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        # init publishers and subscribers
        self._crc = CRC()
        self._cmd_sub = ChannelSubscriber(self._config['topics']['low_cmd_in'], LowCmd_)
        self._state_sub = ChannelSubscriber(self._config['topics']['low_state'], LowState_)
        self._cmd_pub = ChannelPublisher(self._config['topics']['low_cmd_out'], LowCmd_)
        self._cmd_pub.Init()
        # setup async logger
        self._logger = None
        if self._config['logging']['enabled']:
            self._logger = ChunkLogger(
                base_dir=self._config['logging']['base_dir'],
                chunk_prefix=self._config['logging']['chunk_prefix'],
                write_hz=LOG_WRITE_HZ,
                max_queue_size=LOG_MAX_QUEUE_SIZE,
                samples_per_chunk=LOG_SAMPLES_PER_CHUNK
            )
            self._logger.start()

    def start(self) -> None:
        '''Start subscriptions and keep relay alive'''
        self._validate_mode()
        self._running = True

        self._state_sub.Init(self._on_low_state, 10)
        self._cmd_sub.Init(self._on_low_cmd, 10)
        self._log_event({'event': 'relay_started', 'mode': self._config['mode']})

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
        '''Validate config mode and raise if unsupported'''
        mode = self._config['mode'].strip().lower()
        if mode == 'full_body_mode':
            return
        if mode == 'split_mode':
            raise NotImplementedError('mode split_mode is reserved for future dual-low_state support')
        raise ValueError(f'unknown mode: {self._config["mode"]}')

    def _on_low_cmd(self, msg: LowCmd_) -> None:
        '''Callback for incoming low_cmd'''
        with self._lock:
            self._last_cmd = msg

            # if estop is already triggered, just publish estop command
            if self._estop:
                out = make_estop_cmd(msg)
                self._publish_checked(out, source='estop_passthrough')
                return

            # validate and clip command, trigger estop if validation fails
            try:
                out, clipped = clip_low_cmd(msg, self._config['limits'])
            except CmdValidationError as e:
                self._trigger_estop(f'command validation failed: {e}')
                out = make_estop_cmd(msg)
                self._publish_checked(out, source='invalid_cmd_estop')
                return

            # publish checked command
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
                'kp_cmd': self._last_kp_cmd.copy(),
                'kd_cmd': self._last_kd_cmd.copy(),
            }
        )

        with self._lock:
            if self._estop:
                return
            try:
                check_estop_limits(msg, self._config['limits'])
            except EStopTriggered as e:
                print(f'q_cmd: {self._last_q_cmd}')
                print(f'dq_cmd: {self._last_dq_cmd}')
                print(f'tau_cmd: {self._last_tau_cmd}')
                print(f'q: {q}')
                print(f'dq: {dq}')
                print(f'ddq: {ddq}')
                self._trigger_estop(str(e))
                out = make_estop_cmd(self._last_cmd)
                self._publish_checked(out, source='state_estop')

    def _publish_checked(self, msg: LowCmd_, source: str, clipped: int = 0) -> None:
        msg.crc = self._crc.Crc(msg)
        self._cmd_pub.Write(msg)

        self._last_q_cmd = np.asarray([float(msg.motor_cmd[i].q) for i in range(MOTOR_COUNT)], dtype=np.float32)
        self._last_dq_cmd = np.asarray([float(msg.motor_cmd[i].dq) for i in range(MOTOR_COUNT)], dtype=np.float32)
        self._last_tau_cmd = np.asarray([float(msg.motor_cmd[i].tau) for i in range(MOTOR_COUNT)], dtype=np.float32)
        self._last_kp_cmd = np.asarray([float(msg.motor_cmd[i].kp) for i in range(MOTOR_COUNT)], dtype=np.float32)
        self._last_kd_cmd = np.asarray([float(msg.motor_cmd[i].kd) for i in range(MOTOR_COUNT)], dtype=np.float32)

    def _trigger_estop(self, reason: str) -> None:
        if self._estop:
            return
        self._estop = True
        self._estop_reason = reason
        self._running = False
        self._log_event({'event': 'estop_triggered', 'reason': reason})

    def _log_event(self, data: dict[str, Any]) -> None:
        print(json.dumps(data, sort_keys=True))

    def _log_sample(self, sample: dict[str, Any]) -> None:
        if self._logger:
            self._logger.log_sample(sample)
