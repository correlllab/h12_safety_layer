'''CLI entrypoint for the H12 safety layer'''

import argparse

import sys
from pathlib import Path

# put project root in sys.path for imports
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from h12_safety_layer.core.config import load_config
from h12_safety_layer.core.safety_layer import SafetyLayer


def main(config_name: str) -> None:
    '''Start the full-body low_cmd safety relay'''
    config_path = PROJECT_ROOT / 'config' / config_name
    config = load_config(config_path)
    safety_layer = SafetyLayer(config)

    try:
        safety_layer.start()
    except KeyboardInterrupt:
        print('KeyboardInterrupt received, stopping safety layer...')
    finally:
        safety_layer.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='h12 low_cmd safety relay')
    parser.add_argument(
        '--config',
        type=str,
        default='default_safety.yaml',
        help='Path to yaml safety config',
    )
    args = parser.parse_args()
    main(args.config)
