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

def _ensure_yaml_extension(config_path: Path) -> Path:
    '''Append .yaml when config path has no suffix'''
    if config_path.suffix:
        return config_path
    return config_path.with_suffix('.yaml')

def _resolve_config_path(config_name: str) -> Path:
    '''Resolve config from either bare name or config/<name> path'''
    config_path = _ensure_yaml_extension(Path(config_name))
    if config_path.is_absolute():
        return config_path
    if config_path.parts and config_path.parts[0] == 'config':
        return PROJECT_ROOT / config_path
    return PROJECT_ROOT / 'config' / config_path

def main(config_name: str) -> None:
    '''Start the full-body low_cmd safety relay'''
    config_path = _resolve_config_path(config_name)
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
        default='default_safety_full.yaml',
        help='Config name or path, e.g. default_safety_full.yaml or config/default_safety_full.yaml',
    )
    args = parser.parse_args()
    main(args.config)
