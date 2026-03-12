'''CLI entrypoint for the H12 safety layer'''

import argparse
import signal
import threading
import time
import sys
from pathlib import Path
from typing import Any


# put project root in sys.path for imports
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from h12_safety_layer.core.config import load_config
from h12_safety_layer.core.safety_layer import SafetyLayer


def main(config_path: str) -> None:
    '''Start the full-body low_cmd safety relay'''
    config = load_config(config_path)
    safety_layer = SafetyLayer(config)

    stop_event = threading.Event()

    def _handle_signal(signum: int, _frame: Any) -> None:
        _ = signum
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    thread = threading.Thread(target=safety_layer.start, name='relay_main', daemon=True)
    thread.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.2)
    finally:
        safety_layer.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='h12 low_cmd safety relay')
    parser.add_argument(
        '--config',
        type=str,
        default=str(PROJECT_ROOT / 'config' / 'default_safety.yaml'),
        help='Path to yaml safety config',
    )
    args = parser.parse_args()
    main(args.config)
