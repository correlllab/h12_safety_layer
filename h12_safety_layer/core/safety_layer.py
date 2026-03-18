'''Safety layer for full-body H12 control'''

import copy
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
SPLIT_UPPER_START = 12


class SafetyLayer:
    '''Relay low_cmd with validation, clipping, estop, and async logging'''
    def __init__(self, config: dict[str, Any]):
        self._config = config
        self._lock = threading.Lock()
        self._estop = False
        self._estop_reason = ''
        self._running = False
        # parse mode
        self._mode = self._config['mode'].strip().lower()
        # init DDS network interface
        if self._config['network']['interface']:
            ChannelFactoryInitialize(self._config['network']['domain_id'],
                                     self._config['network']['interface'])
        else:
            ChannelFactoryInitialize(self._config['network']['domain_id'])
        # store last command
        self._last_cmd = LowCmdDefault()
        self._last_q_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_dq_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_tau_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_kp_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._last_kd_cmd = np.zeros((MOTOR_COUNT,), dtype=np.float32)
        self._desired_cmd_full = make_estop_cmd(self._last_cmd)
        self._desired_cmd_lower = make_estop_cmd(self._last_cmd)
        self._desired_cmd_upper = make_estop_cmd(self._last_cmd)
        # init publishers and subscribers
        self._crc = CRC()
        self._cmd_sub_full = None
        self._cmd_sub_lower = None
        self._cmd_sub_upper = None
        self._state_sub = ChannelSubscriber(self._config['topics']['low_state'], LowState_)
        self._cmd_pub = ChannelPublisher(self._config['topics']['low_cmd_out'], LowCmd_)
        self._publish_hz = float(self._config['control']['publish_hz'])
        self._publisher_thread = threading.Thread(
            target=self._publisher_loop, name='low_cmd_publisher', daemon=True
        )
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
        # initialize subscribers and publishers
        self._init_cmd_subscribers()
        self._state_sub.Init(self._on_low_state, 10)
        self._cmd_pub.Init()
        self._publisher_thread.start()
        # successfully started, log event
        self._log_event({'event': 'relay_started', 'mode': self._mode})

        while self._running:
            time.sleep(0.2)

    def stop(self) -> None:
        '''Stop relay and flush logger'''
        self._running = False
        self._publisher_thread.join(timeout=1.0)
        self._publisher_thread = None
        self._close_cmd_subscribers()
        self._state_sub.Close()
        self._cmd_pub.Close()
        self._log_event({'event': 'relay_stopped'})
        if self._logger:
            self._logger.stop()

    def _validate_mode(self) -> None:
        '''Validate config mode and raise if unsupported'''
        if self._mode == 'full_body_mode':
            return
        if self._mode == 'split_mode':
            return
        raise ValueError(f'unknown mode: {self._config["mode"]}')

    def _init_cmd_subscribers(self) -> None:
        '''Init low_cmd subscribers based on configured mode'''
        if self._mode == 'full_body_mode':
            self._cmd_sub_full = ChannelSubscriber(self._config['topics']['low_cmd_in'], LowCmd_)
            self._cmd_sub_full.Init(self._on_low_cmd_full, 10)
        # split mode
        else:
            self._cmd_sub_lower = ChannelSubscriber(self._config['topics']['low_cmd_lower_in'], LowCmd_)
            self._cmd_sub_upper = ChannelSubscriber(self._config['topics']['low_cmd_upper_in'], LowCmd_)
            self._cmd_sub_lower.Init(self._on_low_cmd_lower, 10)
            self._cmd_sub_upper.Init(self._on_low_cmd_upper, 10)

    def _close_cmd_subscribers(self) -> None:
        '''Close any active low_cmd subscribers'''
        if self._cmd_sub_full is not None:
            self._cmd_sub_full.Close()
            self._cmd_sub_full = None
        if self._cmd_sub_lower is not None:
            self._cmd_sub_lower.Close()
            self._cmd_sub_lower = None
        if self._cmd_sub_upper is not None:
            self._cmd_sub_upper.Close()
            self._cmd_sub_upper = None

    def _clip_cmd_or_estop(self, msg: LowCmd_, source: str) -> LowCmd_ | None:
        '''Clip incoming command and trigger estop on validation failures'''
        self._last_cmd = msg
        if self._estop:
            return None

        try:
            return clip_low_cmd(msg, self._config['limits'])
        except CmdValidationError as e:
            self._trigger_estop(f'Command validation failed on {source}: {e}')
            self._desired_cmd_full = make_estop_cmd(self._last_cmd)
            self._desired_cmd_lower = make_estop_cmd(self._last_cmd)
            self._desired_cmd_upper = make_estop_cmd(self._last_cmd)
            return None

    def _on_low_cmd_full(self, msg: LowCmd_) -> None:
        '''Update full-body desired command from incoming low_cmd'''
        with self._lock:
            out = self._clip_cmd_or_estop(msg, source='full_body_mode')
            if out is not None:
                self._desired_cmd_full = out

    def _on_low_cmd_lower(self, msg: LowCmd_) -> None:
        '''Update split-mode lower-body desired command from incoming low_cmd'''
        with self._lock:
            out = self._clip_cmd_or_estop(msg, source='split_mode_lower')
            if out is not None:
                self._desired_cmd_lower = out

    def _on_low_cmd_upper(self, msg: LowCmd_) -> None:
        '''Update split-mode upper-body desired command from incoming low_cmd'''
        with self._lock:
            out = self._clip_cmd_or_estop(msg, source='split_mode_upper')
            if out is not None:
                self._desired_cmd_upper = out

    def _build_split_cmd_locked(self) -> LowCmd_:
        '''
        Merge split commands into one full-body command
        (lock acquired in caller method)
        '''
        merged = copy.deepcopy(self._desired_cmd_upper)
        # keep torso+arms (12:27) from upper and overwrite legs (0:12) from lower
        for i in range(SPLIT_UPPER_START):
            merged.motor_cmd[i] = copy.deepcopy(self._desired_cmd_lower.motor_cmd[i])
        return merged

    def _get_publish_cmd_locked(self) -> LowCmd_:
        '''
        Get command to publish in current mode
        (lock acquired in caller method)
        '''
        if self._mode == 'full_body_mode':
            return copy.deepcopy(self._desired_cmd_full)
        return self._build_split_cmd_locked()

    def _publisher_loop(self) -> None:
        dt = 1.0 / self._publish_hz
        while self._running:
            start_time = time.time()
            with self._lock:
                if self._estop:
                    out = make_estop_cmd(self._last_cmd)
                else:
                    out = self._get_publish_cmd_locked()
            self._publish_checked(out)
            time.sleep(max(0.0, dt - (time.time() - start_time)))

    def _on_low_state(self, msg: LowState_) -> None:
        '''Check state limits for estop and log sample'''
        with self._lock:
            if self._estop:
                return
            try:
                check_estop_limits(msg, self._config['limits'])
            except EStopTriggered as e:
                q = np.asarray([float(msg.motor_state[i].q) for i in range(MOTOR_COUNT)], dtype=np.float32)
                dq = np.asarray([float(msg.motor_state[i].dq) for i in range(MOTOR_COUNT)], dtype=np.float32)
                ddq = np.asarray([float(msg.motor_state[i].ddq) for i in range(MOTOR_COUNT)], dtype=np.float32)
                print(f'q_cmd: {self._last_q_cmd}')
                print(f'dq_cmd: {self._last_dq_cmd}')
                print(f'tau_cmd: {self._last_tau_cmd}')
                print(f'q: {q}')
                print(f'dq: {dq}')
                print(f'ddq: {ddq}')
                self._trigger_estop(str(e))
                self._desired_cmd_full = make_estop_cmd(self._last_cmd)
                self._desired_cmd_lower = make_estop_cmd(self._last_cmd)
                self._desired_cmd_upper = make_estop_cmd(self._last_cmd)

        self._log_sample(msg)

    def _publish_checked(self, msg: LowCmd_) -> None:
        # write message
        msg.crc = self._crc.Crc(msg)
        self._cmd_pub.Write(msg)
        # write local copy for logging
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
        self._log_event({'event': 'estop_triggered', 'reason': reason})

    def _log_event(self, data: dict[str, Any]) -> None:
        print(json.dumps(data, sort_keys=True))

    def _log_sample(self, msg: LowState_) -> None:
        '''Build logging dict from low_state message and log via logger'''
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

        sample = {
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

        if self._logger:
            self._logger.log_sample(sample)
