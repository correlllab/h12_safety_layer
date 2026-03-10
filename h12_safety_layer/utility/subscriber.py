'''
Unitree SDK2 channel subscriber example.
This module subscribes to `std_msgs/String_` payloads from `demo/topic`.
'''

from __future__ import annotations

import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


def on_message(msg: String_) -> None:
    '''Print incoming payload from the subscribed topic'''
    print(f'[SUB] {msg.data}')


def main() -> None:
    '''Initialize a channel subscriber and wait for callbacks'''
    ChannelFactoryInitialize(0)

    subscriber = ChannelSubscriber('demo/topic', String_)
    subscriber.Init(on_message, 10)

    while True:
        time.sleep(1.0)


if __name__ == '__main__':
    main()
