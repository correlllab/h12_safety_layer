'''Safety layer for full-body H12 control'''

import copy
import time
import json
import threading
from typing import Any

import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from h12_safety_layer.core.chunk_logger import ChunkLogger
from h12_safety_layer.core.config import MOTOR_COUNT
from h12_safety_layer.core.estop_subscriber import EStopSubscriber
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
# Once an upper-body publisher has produced at least one command, treat
# longer-than-this gaps as a fault and escalate to estop. Designed to detect
# frame_task_server crashes mid-run; does not fire if upper is never published
# at all (lower-only deployments stay safe). This is only the DEFAULT: the
# effective timeout comes from config estop.upper_stale_sec (<= 0 disables it).
UPPER_STALE_ESTOP_SECONDS = 2.0


class SafetyLayer:
    '''Relay low_cmd with validation, clipping, estop, and async logging'''
    def __init__(self, config: dict[str, Any]):
        self._config = config
        self._lock = threading.Lock()
        self._estop = False
        self._estop_reason = ''
        # created while holding _lock and emitted only after the first zero
        # command has been written, so stdout cannot delay estop publication
        self._pending_estop_report: dict[str, Any] | None = None
        self._running = False
        # parse mode
        self._mode = self._config['mode'].strip().lower()
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
        self._last_upper_msg_time: float | None = None
        # init publishers and subscribers
        self._crc = CRC()
        self._cmd_sub_full = None
        self._cmd_sub_lower = None
        self._cmd_sub_upper = None
        self._state_sub = ChannelSubscriber(
            self._config['topics']['low_state'], LowState_
        )
        self._cmd_pub = ChannelPublisher(self._config['topics']['low_cmd_out'], LowCmd_)
        self._publish_hz = float(self._config['control']['publish_hz'])
        self._publisher_thread = threading.Thread(
            target=self._publisher_loop, name='low_cmd_publisher', daemon=True
        )
        # estop config
        estop_config = self._config['estop']
        self._enable_estop = bool(estop_config['enabled'])
        # upper-body stale watchdog timeout (split_mode only); <= 0 disables it
        self._upper_stale_sec = float(
            estop_config.get('upper_stale_sec', UPPER_STALE_ESTOP_SECONDS)
        )
        # estop subscriber and monitor thread
        self._estop_poll_hz = float(estop_config['poll_hz'])
        self._estop_sub = None
        self._estop_thread = None
        if self._enable_estop:
            self._estop_sub = EStopSubscriber(
                topic=estop_config['estop_topic'],
                poll_hz=self._estop_poll_hz,
            )
            self._estop_thread = threading.Thread(
                target=self._estop_monitor_loop, name='estop_monitor', daemon=True
            )
        # setup async logger
        self._logger = None
        if self._config['logging']['enabled']:
            self._logger = ChunkLogger(
                base_dir=self._config['logging']['base_dir'],
                chunk_prefix=self._config['logging']['chunk_prefix'],
                write_hz=LOG_WRITE_HZ,
                max_queue_size=LOG_MAX_QUEUE_SIZE,
                samples_per_chunk=LOG_SAMPLES_PER_CHUNK,
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
        # start estop monitor thread
        if self._estop_thread is not None:
            self._estop_thread.start()
        # successfully started, log event
        self._log_event({'event': 'relay_started', 'mode': self._mode})

        while self._running:
            time.sleep(0.2)

    def stop(self) -> None:
        '''Stop relay and flush logger'''
        self._running = False
        self._publisher_thread.join(timeout=1.0)
        if self._estop_thread is not None:
            self._estop_thread.join(timeout=1.0)
        if self._estop_sub is not None:
            self._estop_sub.close()
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
        mode_name = self._config['mode']
        raise ValueError(f'Unknown mode: {mode_name}')

    def _init_cmd_subscribers(self) -> None:
        '''Init low_cmd subscribers based on configured mode'''
        if self._mode == 'full_body_mode':
            self._cmd_sub_full = ChannelSubscriber(
                self._config['topics']['low_cmd_in'], LowCmd_
            )
            self._cmd_sub_full.Init(self._on_low_cmd_full, 10)
        # split mode
        else:
            self._cmd_sub_lower = ChannelSubscriber(
                self._config['topics']['low_cmd_lower_in'], LowCmd_
            )
            self._cmd_sub_upper = ChannelSubscriber(
                self._config['topics']['low_cmd_upper_in'], LowCmd_
            )
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
                self._last_upper_msg_time = time.time()

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
                    self._check_upper_watchdog_locked(start_time)
                    out = self._get_publish_cmd_locked()
            action_ms = self._publish_checked(out)
            report = self._get_estop_trigger_report(action_ms)
            if report is not None:
                self._log_event(report)
            time.sleep(max(0.0, dt - (time.time() - start_time)))

    def _check_upper_watchdog_locked(self, now: float) -> None:
        '''
        Escalate to estop if the upper-body publisher fell silent after having
        produced at least one command. Only active in split_mode; never fires
        before the first upper message arrives, so lower-only deployments are
        unaffected.
        '''
        if self._mode != 'split_mode':
            return
        if self._upper_stale_sec <= 0.0:
            return
        if self._last_upper_msg_time is None:
            return
        if now - self._last_upper_msg_time > self._upper_stale_sec:
            self._trigger_estop(
                f'Upper-body command stale for '
                f'{now - self._last_upper_msg_time:.2f}s '
                f'(> {self._upper_stale_sec}s)'
            )

    def _estop_monitor_loop(self) -> None:
        if self._estop_sub is None:
            return

        dt = 1.0 / self._estop_poll_hz
        while self._running:
            start_time = time.time()
            should_trigger = False
            reason = ''
            with self._lock:
                if self._estop:
                    pass
                else:
                    status = self._estop_sub.latest
                    if status is not None:
                        # estop if triggered or unplugged
                        if status.triggered:
                            should_trigger = True
                            reason = 'External estop triggered'
                        elif not status.plugged_in:
                            should_trigger = True
                            reason = 'External estop unplugged'
                    if should_trigger:
                        self._trigger_estop(
                            reason, trigger_stamp_ms=status.stamp_ms
                        )
            # sleep to maintain poll rate
            time.sleep(max(0.0, dt - (time.time() - start_time)))

    def _on_low_state(self, msg: LowState_) -> None:
        '''Check state limits for estop and log sample'''
        diagnostic: dict[str, np.ndarray] | None = None
        with self._lock:
            if self._estop:
                return
            try:
                check_estop_limits(msg, self._config['limits'])
            except EStopTriggered as e:
                q = np.asarray(
                    [float(msg.motor_state[i].q) for i in range(MOTOR_COUNT)],
                    dtype=np.float32,
                )
                dq = np.asarray(
                    [float(msg.motor_state[i].dq) for i in range(MOTOR_COUNT)],
                    dtype=np.float32,
                )
                ddq = np.asarray(
                    [float(msg.motor_state[i].ddq) for i in range(MOTOR_COUNT)],
                    dtype=np.float32,
                )
                diagnostic = {
                    'q_cmd': self._last_q_cmd.copy(),
                    'dq_cmd': self._last_dq_cmd.copy(),
                    'tau_cmd': self._last_tau_cmd.copy(),
                    'q': q,
                    'dq': dq,
                    'ddq': ddq,
                }
                self._trigger_estop(str(e))

        if diagnostic is not None:
            for name, values in diagnostic.items():
                print(f'{name}: {values}')
        self._log_sample(msg)

    def _publish_checked(self, msg: LowCmd_) -> int:
        # write message
        msg.crc = self._crc.Crc(msg)
        self._cmd_pub.Write(msg)
        action_ms = int(time.time() * 1000)
        # write local copy for logging
        self._last_q_cmd = np.asarray(
            [float(msg.motor_cmd[i].q) for i in range(MOTOR_COUNT)], dtype=np.float32
        )
        self._last_dq_cmd = np.asarray(
            [float(msg.motor_cmd[i].dq) for i in range(MOTOR_COUNT)], dtype=np.float32
        )
        self._last_tau_cmd = np.asarray(
            [float(msg.motor_cmd[i].tau) for i in range(MOTOR_COUNT)], dtype=np.float32
        )
        self._last_kp_cmd = np.asarray(
            [float(msg.motor_cmd[i].kp) for i in range(MOTOR_COUNT)], dtype=np.float32
        )
        self._last_kd_cmd = np.asarray(
            [float(msg.motor_cmd[i].kd) for i in range(MOTOR_COUNT)], dtype=np.float32
        )
        return action_ms

    def _trigger_estop(
        self, reason: str, trigger_stamp_ms: int | None = None
    ) -> None:
        '''Set estop state while holding _lock; defer its log until command write'''
        if self._estop:
            return
        self._estop = True
        self._estop_reason = reason
        decision_ms = int(time.time() * 1000)
        self._pending_estop_report = {
            'event': 'estop_command_written',
            'reason': reason,
            'trigger_stamp_ms': trigger_stamp_ms,
            'decision_ms': decision_ms,
        }
        # overwrite desired commands with estop command
        self._desired_cmd_full = make_estop_cmd(self._last_cmd)
        self._desired_cmd_lower = make_estop_cmd(self._last_cmd)
        self._desired_cmd_upper = make_estop_cmd(self._last_cmd)

    def _get_estop_trigger_report(
        self, action_ms: int
    ) -> dict[str, Any] | None:
        '''Build the one-time timing report after the estop command is written'''
        with self._lock:
            report = self._pending_estop_report
            self._pending_estop_report = None

        if report is None:
            return None

        report['action_ms'] = action_ms
        report['decision_to_action_ms'] = action_ms - report['decision_ms']
        trigger_stamp_ms = report['trigger_stamp_ms']
        if trigger_stamp_ms is not None:
            # this requires the estop publisher and safety host clocks to be
            # synchronized, and a negative value signals clock skew
            report['trigger_to_action_ms'] = action_ms - trigger_stamp_ms
        return report

    def _log_event(self, data: dict[str, Any]) -> None:
        print(json.dumps(data, sort_keys=True))

    def _log_sample(self, msg: LowState_) -> None:
        '''Build logging dict from low_state message and log via logger'''
        time_stamp = time.time()

        mode = np.asarray(
            [int(msg.motor_state[i].mode) for i in range(MOTOR_COUNT)], dtype=np.uint8
        )
        q = np.asarray(
            [float(msg.motor_state[i].q) for i in range(MOTOR_COUNT)], dtype=np.float32
        )
        dq = np.asarray(
            [float(msg.motor_state[i].dq) for i in range(MOTOR_COUNT)], dtype=np.float32
        )
        ddq = np.asarray(
            [float(msg.motor_state[i].ddq) for i in range(MOTOR_COUNT)],
            dtype=np.float32,
        )
        tau_est = np.asarray(
            [float(msg.motor_state[i].tau_est) for i in range(MOTOR_COUNT)],
            dtype=np.float32,
        )
        temperature = np.asarray(
            [msg.motor_state[i].temperature for i in range(MOTOR_COUNT)], dtype=np.int16
        )
        vol = np.asarray(
            [float(msg.motor_state[i].vol) for i in range(MOTOR_COUNT)],
            dtype=np.float32,
        )
        sensor = np.asarray(
            [msg.motor_state[i].sensor for i in range(MOTOR_COUNT)], dtype=np.uint32
        )
        motorstate = np.asarray(
            [int(msg.motor_state[i].motorstate) for i in range(MOTOR_COUNT)],
            dtype=np.uint32,
        )
        reserve = np.asarray(
            [msg.motor_state[i].reserve for i in range(MOTOR_COUNT)], dtype=np.uint32
        )

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
