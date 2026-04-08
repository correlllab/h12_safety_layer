'''Simple CLI to monitor estop DDS status'''

import argparse
import sys
import time
from pathlib import Path


# put project root in sys.path for imports
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from h12_safety_layer.core.estop_subscriber import EStopSubscriber


def _format_status(subscriber: EStopSubscriber) -> str:
    '''Build one status line from latest estop sample'''
    status = subscriber.latest
    if status is None:
        return 'triggered=False plugged_in=False seq=-1 stamp_ms=-1 waiting_sample=True'
    return (
        f'triggered={status.triggered} '
        f'plugged_in={status.plugged_in} '
        f'seq={status.sequence_id} '
        f'stamp_ms={status.stamp_ms} '
    )


def main(topic: str, poll_hz: float, print_hz: float) -> None:
    '''Start estop subscriber and print periodic status'''
    subscriber = EStopSubscriber(topic=topic, poll_hz=poll_hz)
    print_dt = 1.0 / print_hz

    try:
        while True:
            line = _format_status(subscriber)
            if subscriber.last_error is not None:
                line = f'{line} last_error={subscriber.last_error}'
            print(line)
            time.sleep(print_dt)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Monitor raw estop DDS status')
    parser.add_argument('--topic', type=str, default='h12/estop_status_raw')
    parser.add_argument('--poll-hz', type=float, default=1000.0)
    parser.add_argument('--print-hz', type=float, default=100.0)
    args = parser.parse_args()

    main(
        topic=args.topic,
        poll_hz=args.poll_hz,
        print_hz=args.print_hz,
    )
