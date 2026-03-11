'''
Unitree SDK2 channel subscriber example.
This module subscribes to `builtin_interfaces/Time_` timestamps from `demo/topic`
and prints the one-way latency derived from the embedded send timestamp.
'''

from __future__ import annotations

import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_

_seq = 0


def on_message(msg: Time_) -> None:
    '''Compute and print one-way latency from the received timestamp'''
    global _seq
    send_t = msg.sec + msg.nanosec * 1e-9
    latency_ms = (time.time() - send_t) * 1000.0
    print(f'[SUB] seq={_seq}  latency={latency_ms:.3f} ms')
    _seq += 1


def main() -> None:
    '''Initialize a channel subscriber and wait for callbacks'''
    ChannelFactoryInitialize(0)

    subscriber = ChannelSubscriber('demo/topic', Time_)
    subscriber.Init(on_message, 10)

    while True:
        time.sleep(1.0)


if __name__ == '__main__':
    main()
