'''ROS2 executable entrypoint for the H12 safety layer'''

import argparse
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from h12_safety_layer.core.config import init_channel_factory, load_config
from h12_safety_layer.core.safety_layer import SafetyLayer

PACKAGE_NAME = 'h12_safety_layer'

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
    share_dir = Path(get_package_share_directory(PACKAGE_NAME))
    if config_path.parts and config_path.parts[0] == 'config':
        return share_dir / config_path
    return share_dir / 'config' / config_path

class SafetyNode(Node):
    '''ROS2 node that manages SafetyLayer lifecycle'''
    def __init__(self, config_name: str = 'default_safety_full.yaml') -> None:
        super().__init__('safety_node')
        config_path = _resolve_config_path(config_name)
        config = load_config(config_path)
        init_channel_factory(config)
        self._safety_layer = SafetyLayer(config)
        self._worker = threading.Thread(
            target=self._run_safety_layer,
            name='safety_layer_worker',
            daemon=True,
        )
        self._worker.start()
        self.get_logger().info(f'Loaded config: {config_path}')

    def _run_safety_layer(self) -> None:
        try:
            self._safety_layer.start()
        except Exception as exc:  # broad catch to keep node alive for shutdown path
            self.get_logger().error(f'Safety layer exited with error: {exc}')

    def destroy_node(self) -> bool:
        self._safety_layer.stop()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        return super().destroy_node()

def main(args: list[str] | None = None) -> None:
    '''Parse CLI args, start ROS2 node, and spin until shutdown'''
    parser = argparse.ArgumentParser(description='h12 low_cmd safety relay')
    parser.add_argument(
        '--config',
        type=str,
        default='default_safety_full.yaml',
        help='Config name or path, e.g. default_safety_full.yaml or config/default_safety_full.yaml',
    )
    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = SafetyNode(config_name=parsed_args.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
