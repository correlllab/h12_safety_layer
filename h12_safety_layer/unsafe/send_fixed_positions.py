'''Dangerous debug script that sends a fixed joint-limit configuration for 1 second'''

import argparse
import time
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

from h12_safety_layer.core.joint_limits import MOTOR_COUNT, URDF_POSITION_LIMITS


SAFETY_TOPIC = 'rt/safety/lowcmd_in'
RAW_TOPIC = 'rt/lowcmd'

ARM_WAIST_KPS = [
    150,
    300, 300, 300, 300, 50, 50, 50,
    300, 300, 300, 300, 50, 50, 50,
]
ARM_WAIST_KDS = [
    3,
    2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2,
]
LEGS_KPS = [
    300, 300, 300, 300, 80, 200,
    300, 300, 300, 300, 80, 200,
]
LEGS_KDS = [
    8.0, 8.0, 8.0, 8.0, 3.0, 5.0,
    8.0, 8.0, 8.0, 8.0, 3.0, 5.0,
]

FIXED_Q_BLEND = 0.5
FIXED_TARGET_Q = np.asarray(
    [
        float(lim['low']) + FIXED_Q_BLEND * (float(lim['high']) - float(lim['low']))
        for lim in URDF_POSITION_LIMITS
    ],
    dtype=np.float32,
)


def _print_danger_warnings(topic: str) -> None:
    print('')
    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    print('!!! EXTREME DANGER: THIS SCRIPT COMMANDS JOINTS DIRECTLY        !!!')
    print('!!! THIS IS FOR DEBUG ONLY                                        !!!')
    print('!!! DO NOT RUN ON A REAL ROBOT UNLESS YOU ACCEPT FULL RISK       !!!')
    print('!!! COLLISION, FALL, HARDWARE DAMAGE, OR INJURY MAY OCCUR        !!!')
    print('!!! YOU ARE RESPONSIBLE FOR ALL CONSEQUENCES                      !!!')
    print('!!! TARGET TOPIC:', topic)
    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    print('')


def _confirm_or_abort() -> None:
    step_1 = input('type y: ').strip()
    if step_1 != 'y':
        raise RuntimeError('aborted by user')

    step_2 = input('type understand: ').strip()
    if step_2 != 'understand':
        raise RuntimeError('aborted by user')

    step_3 = input('type I fully understand and decide to run: ').strip()
    if step_3 != 'I fully understand and decide to run':
        raise RuntimeError('aborted by user')


def _set_joint_gains(cmd: LowCmd_) -> None:
    for i in range(12):
        cmd.motor_cmd[i].kp = float(LEGS_KPS[i])
        cmd.motor_cmd[i].kd = float(LEGS_KDS[i])

    for i in range(15):
        idx = 12 + i
        cmd.motor_cmd[idx].kp = float(ARM_WAIST_KPS[i])
        cmd.motor_cmd[idx].kd = float(ARM_WAIST_KDS[i])


def _fill_fixed_limit_command(cmd: LowCmd_, q_target: np.ndarray) -> None:
    for i in range(MOTOR_COUNT):
        motor = cmd.motor_cmd[i]
        motor.mode = 1
        motor.q = float(q_target[i])
        motor.dq = 0.0
        motor.tau = 0.0


def _publish_for_one_second(topic: str, domain_id: int, interface: str | None, hz: float) -> None:
    if interface:
        ChannelFactoryInitialize(domain_id, interface)
    else:
        ChannelFactoryInitialize(domain_id)

    publisher = ChannelPublisher(topic, LowCmd_)
    publisher.Init()

    crc = CRC()
    cmd = LowCmdDefault()
    _set_joint_gains(cmd)

    dt = 1.0 / hz
    end_time = time.time() + 1.0
    sent = 0

    while time.time() < end_time:
        _fill_fixed_limit_command(cmd, FIXED_TARGET_Q)
        cmd.crc = crc.Crc(cmd)
        publisher.Write(cmd)
        sent += 1
        time.sleep(dt)

    print(f'Published {sent} fixed-limit configuration commands to {topic}')


def main(topic: str, domain_id: int, interface: str | None,hz: float) -> None:
    _print_danger_warnings(topic)
    _confirm_or_abort()
    _publish_for_one_second(topic=topic, domain_id=domain_id, interface=interface, hz=hz)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dangerous debug: send fixed joint-limit configuration for 1 second')
    parser.add_argument('--domain-id', type=int, default=0, help='DDS domain id')
    parser.add_argument('--interface', type=str, default='', help='DDS network interface')
    parser.add_argument('--hz', type=float, default=50.0, help='Publish rate in Hz')

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--safety', action='store_true', help='Publish to rt/safety/lowcmd_in')
    group.add_argument('--raw', action='store_true', help='Publish to rt/lowcmd')

    args = parser.parse_args()
    topic = SAFETY_TOPIC if args.safety else RAW_TOPIC
    interface = args.interface.strip() or None
    main(topic=topic, domain_id=args.domain_id, interface=interface, hz=args.hz)
