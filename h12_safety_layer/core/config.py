'''YAML config loader'''

import yaml
import numpy as np
from typing import Any
from pathlib import Path

from h12_safety_layer.core.joint_limits import (
    MOTOR_COUNT,
    JOINT_NAMES,
    URDF_POSITION_LIMITS,
    URDF_TORQUE_LIMITS,
    URDF_VELOCITY_LIMITS
)


def _load_yaml(path: Path) -> dict[str, Any]:
    '''Load yaml file and ensure it's a dict'''
    with path.open('r', encoding='utf-8') as handle:
        loaded = yaml.safe_load(handle)
    if not isinstance(loaded, dict):
        raise ValueError('config root must be a mapping')
    return loaded

def _load_joint_config(value: Any, field_name: str) -> np.ndarray:
    '''Expand a scalar value to a vector of MOTOR_COUNT, or validate list input'''
    # single value for all joints
    if isinstance(value, (int, float)):
        return np.full((MOTOR_COUNT,), float(value), dtype=np.float64)
    # list of values for each joint
    if isinstance(value, list) and len(value) == MOTOR_COUNT:
        return np.asarray(value, dtype=np.float64)
    # mismatch
    raise ValueError(f'{field_name} must be a number or a list with {MOTOR_COUNT} entries')

def _derive_q_limits(position_offset: np.ndarray) -> np.ndarray:
    '''Derive low and high position limits for each joint from URDF limits and config offset'''
    q_limits = np.zeros((MOTOR_COUNT, 2), dtype=np.float64)
    for i in range(MOTOR_COUNT):
        low = float(URDF_POSITION_LIMITS[i]['low']) + position_offset[i]
        high = float(URDF_POSITION_LIMITS[i]['high']) - position_offset[i]
        if low >= high:
            raise ValueError(f'position_offset too large for joint {i} {JOINT_NAMES[i]}')
        q_limits[i, 0] = low
        q_limits[i, 1] = high
    return q_limits

def _process_clip_limits(policy: dict[str, Any]) -> dict[str, np.ndarray]:
    '''Process clip policy from config and derive per-joint limits'''
    position_offset = _load_joint_config(policy.get('position_offset', 0.01), 'position_offset')
    velocity_ratio = _load_joint_config(policy.get('velocity_ratio', 0.5), 'velocity_ratio')
    torque_ratio = _load_joint_config(policy.get('torque_ratio', 0.5), 'torque_ratio')
    kp_max = _load_joint_config(policy.get('kp_max', policy.get('kp_abs', 300.0)), 'kp_max')
    kd_max = _load_joint_config(policy.get('kd_max', policy.get('kd_abs', 30.0)), 'kd_max')
    if np.any(kp_max < 0.0):
        raise ValueError('kp_max must be nonnegative')
    if np.any(kd_max < 0.0):
        raise ValueError('kd_max must be nonnegative')

    return {
        'q_limits': _derive_q_limits(position_offset),
        'dq_limits': np.asarray(URDF_VELOCITY_LIMITS, dtype=np.float64) * velocity_ratio,
        'tau_limits': np.asarray(URDF_TORQUE_LIMITS, dtype=np.float64) * torque_ratio,
        'kp_max': kp_max,
        'kd_max': kd_max,
    }

def _process_estop_limits(policy: dict[str, Any]) -> dict[str, np.ndarray]:
    '''Process estop config and derive per-joint limits'''
    position_offset = _load_joint_config(policy.get('position_offset', 0.01), 'position_offset')
    velocity_ratio = _load_joint_config(policy.get('velocity_ratio', 0.5), 'velocity_ratio')
    torque_ratio = _load_joint_config(policy.get('torque_ratio', 0.5), 'torque_ratio')

    return {
        'q_limits': _derive_q_limits(position_offset),
        'dq_limits': np.asarray(URDF_VELOCITY_LIMITS, dtype=np.float64) * velocity_ratio,
        'tau_limits': np.asarray(URDF_TORQUE_LIMITS, dtype=np.float64) * torque_ratio,
    }

def load_config(path: str | Path) -> dict[str, Any]:
    '''Load YAML config as plain dict'''
    raw = _load_yaml(Path(path))

    topics = raw.get('topics', {})
    network = raw.get('network', {})
    logging = raw.get('logging', {})
    control = raw.get('control', {})
    limits = raw.get('limits', {})

    publish_hz = float(control.get('publish_hz', 500.0))
    if publish_hz <= 0.0:
        raise ValueError('control.publish_hz must be positive')

    clip_policy = limits.get('clip', limits.get('enforce', {}))
    estop_policy = limits.get('estop', {})

    clip_limits = _process_clip_limits(clip_policy)
    estop_limits = _process_estop_limits(estop_policy)

    return {
        'mode': str(raw.get('mode', 'full_body_mode')),
        'topics': {
            'low_cmd_in': str(topics.get('low_cmd_in', 'rt/safety/lowcmd_in')),
            'low_cmd_lower_in': str(topics.get('low_cmd_lower_in', 'rt/safety/lowcmd_lower_in')),
            'low_cmd_upper_in': str(topics.get('low_cmd_upper_in', 'rt/safety/lowcmd_upper_in')),
            'low_cmd_out': str(topics.get('low_cmd_out', 'rt/lowcmd')),
            'low_state': str(topics.get('low_state', 'rt/lowstate')),
        },
        'network': {
            'domain_id': int(network.get('domain_id', 0)),
            'interface': None if network.get('interface') in (None, '') else str(network.get('interface')),
        },
        'control': {
            'publish_hz': publish_hz,
        },
        'limits': {
            'q_clip_limits': clip_limits['q_limits'],
            'dq_clip_limits': clip_limits['dq_limits'],
            'tau_clip_limits': clip_limits['tau_limits'],
            'kp_clip_max': clip_limits['kp_max'],
            'kd_clip_max': clip_limits['kd_max'],
            'q_estop_limits': estop_limits['q_limits'],
            'dq_estop_limits': estop_limits['dq_limits'],
            'tau_estop_limits': estop_limits['tau_limits'],
        },
        'logging': {
            'enabled': bool(logging.get('enabled', True)),
            'base_dir': str(logging.get('base_dir', '/tmp/h12_safety_layer')),
            'chunk_prefix': str(logging.get('chunk_prefix', 'chunk_')),
        },
    }
