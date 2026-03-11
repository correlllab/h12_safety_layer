'''Safety checks for low_cmd and low_state messages'''

import copy
import math
import numpy as np
from typing import Any

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from h12_safety_layer.core.joint_limits import MOTOR_COUNT

class CommandValidationError(Exception):
    '''Command payload contains non-finite values'''

class EStopTriggered(Exception):
    '''E-stop condition was met from low_state feedback'''

def _is_finite(value: float) -> bool:
    return math.isfinite(float(value))

def _clip_abs(value: float, max_abs: float) -> float:
    if value > max_abs:
        return max_abs
    if value < -max_abs:
        return -max_abs
    return value

def clone_cmd(msg: LowCmd_) -> LowCmd_:
    '''Create a deep copy of low_cmd before mutating'''

    return copy.deepcopy(msg)


def _cmd_arrays(msg: LowCmd_) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    q = np.asarray([float(msg.motor_cmd[i].q) for i in range(MOTOR_COUNT)], dtype=np.float64)
    dq = np.asarray([float(msg.motor_cmd[i].dq) for i in range(MOTOR_COUNT)], dtype=np.float64)
    tau = np.asarray([float(msg.motor_cmd[i].tau) for i in range(MOTOR_COUNT)], dtype=np.float64)
    kp = np.asarray([float(msg.motor_cmd[i].kp) for i in range(MOTOR_COUNT)], dtype=np.float64)
    kd = np.asarray([float(msg.motor_cmd[i].kd) for i in range(MOTOR_COUNT)], dtype=np.float64)
    return q, dq, tau, kp, kd


def sanitize_and_clip_command(msg: LowCmd_, limits: dict[str, np.ndarray]) -> tuple[LowCmd_, int]:
    '''Validate finite values and clip command to configured limits'''

    out = clone_cmd(msg)
    q_raw, dq_raw, tau_raw, kp_raw, kd_raw = _cmd_arrays(out)

    if not np.all(np.isfinite(q_raw)):
        raise CommandValidationError('q contains non-finite values')
    if not np.all(np.isfinite(dq_raw)):
        raise CommandValidationError('dq contains non-finite values')
    if not np.all(np.isfinite(tau_raw)):
        raise CommandValidationError('tau contains non-finite values')
    if not np.all(np.isfinite(kp_raw)):
        raise CommandValidationError('kp contains non-finite values')
    if not np.all(np.isfinite(kd_raw)):
        raise CommandValidationError('kd contains non-finite values')

    q_low = limits['q_clip_limits'][:, 0]
    q_high = limits['q_clip_limits'][:, 1]
    q_new = np.clip(q_raw, q_low, q_high)
    dq_new = np.clip(dq_raw, -limits['dq_clip_limits'], limits['dq_clip_limits'])
    tau_new = np.clip(tau_raw, -limits['tau_clip_limits'], limits['tau_clip_limits'])
    kp_new = np.clip(kp_raw, 0.0, limits['kp_clip_max'])
    kd_new = np.clip(kd_raw, 0.0, limits['kd_clip_max'])

    changed = np.logical_or.reduce((
        q_new != q_raw,
        dq_new != dq_raw,
        tau_new != tau_raw,
        kp_new != kp_raw,
        kd_new != kd_raw,
    ))
    clipped_count = int(np.count_nonzero(changed))

    for i in range(MOTOR_COUNT):
        cmd = out.motor_cmd[i]
        cmd.q = float(q_new[i])
        cmd.dq = float(dq_new[i])
        cmd.tau = float(tau_new[i])
        cmd.kp = float(kp_new[i])
        cmd.kd = float(kd_new[i])

    return out, clipped_count


def assert_state_within_estop_limits(msg: LowState_, limits: dict[str, np.ndarray]) -> None:
    '''Raise if low_state exceeds configured estop limits'''

    q = np.asarray([float(msg.motor_state[i].q) for i in range(MOTOR_COUNT)], dtype=np.float64)
    dq = np.asarray([float(msg.motor_state[i].dq) for i in range(MOTOR_COUNT)], dtype=np.float64)
    tau = np.asarray([float(msg.motor_state[i].tau_est) for i in range(MOTOR_COUNT)], dtype=np.float64)

    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)) or not np.all(np.isfinite(tau)):
        raise EStopTriggered('low_state contains non-finite values')

    q_low = limits['q_estop_limits'][:, 0]
    q_high = limits['q_estop_limits'][:, 1]

    bad_q = np.where(np.logical_or(q < q_low, q > q_high))[0]
    if bad_q.size > 0:
        i = int(bad_q[0])
        raise EStopTriggered(f'motor {i} q out of estop range: {q[i]}')

    bad_dq = np.where(np.abs(dq) > limits['dq_estop_limits'])[0]
    if bad_dq.size > 0:
        i = int(bad_dq[0])
        raise EStopTriggered(f'motor {i} dq out of estop range: {dq[i]}')

    bad_tau = np.where(np.abs(tau) > limits['tau_estop_limits'])[0]
    if bad_tau.size > 0:
        i = int(bad_tau[0])
        raise EStopTriggered(f'motor {i} tau out of estop range: {tau[i]}')


def make_estop_cmd_like(template: LowCmd_) -> LowCmd_:
    '''Create an estop command preserving headers while zeroing actuators'''

    out = clone_cmd(template)
    for i in range(MOTOR_COUNT):
        cmd = out.motor_cmd[i]
        cmd.mode = 0
        cmd.q = 0.0
        cmd.dq = 0.0
        cmd.tau = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.0
    return out
