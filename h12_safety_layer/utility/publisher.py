'''
Unitree SDK2 channel publisher example.
This module publishes `std_msgs/String_` payloads to `demo/topic`.
'''

from __future__ import annotations

import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


def main() -> None:
    '''initialize a channel publisher and send one message per second'''
    ChannelFactoryInitialize(0)

    publisher = ChannelPublisher('demo/topic', String_)
    publisher.Init()

    counter = 0
    while True:
        msg = String_(f'Hello from Unitree publisher #{counter}')
        publisher.Write(msg)
        print(f'[PUB] {msg.data}')
        counter += 1
        time.sleep(1.0)


if __name__ == '__main__':
    main()
