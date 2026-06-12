# h12_safety_layer

Safety layer for H1-2 that filters commands, monitors joint states, and enforces clipping and active-stop logic before forwarding to Unitree topics.

## Installation

- This repo depends on [Unitree Python SDK](https://github.com/unitreerobotics/unitree_sdk2_python) to communicate with the robot.
- We rely on this [fork](https://github.com/Oya-Tomo/unitree_sdk2_python) for easy integration with `uv`.
- The `main` branch is meant to be compatible with [Humanoid Simulation](https://github.com/correlllab/Humanoid_Simulation), so place Unitree SDK at `../../../unitree_sdk2_python`.
- The [dev branch](https://github.com/correlllab/h12_safety_layer/tree/dev) is for local testing.

### `uv` Installation

- Easiest way to run scripts in this repo is to use [`uv`](https://docs.astral.sh/uv/getting-started/installation/)
- Commands:

    ```bash
    uv sync # install dependencies for this repo including unitree sdk
    uv run PATH_TO_SCRIPT
    ```

### `pip` Installation

- `requirements.txt` lists dependencies that can be installed by `pip`.
- Commands:

    ```bash
    pip install -r requirements.txt # install dependencies for this repo including unitree sdk
    ```

## Files

- `config/`: YAML configuration files.
- `docs/`: detailed documentation.
- `h12_safety_layer/`: source codes.
    - `core/`: core implementation of the safety layer.
        - `chunk_logger.py` provides logger of robot states and commands.
        - `config.py` parses and loads YAML configuration files.
        - `joint_limits.py` defines joint names and joint limits according to the URDF.
        - `safety_checks.py` defines several safety checks of robot states and commands.
        - `safety_layer.py` implements the safety layer that relays commands, checks safety and logs in the background.
    - `script/`: runnable scripts.
        - `safety_layer_main.py` launches the safety layer.
    - `unsafe/`: unsafe debugging scripts.
        - `send_fixed_positions.py` sends a fixed position command for 1 second.
        - `send_random_positions.py` sends random position commands for 1 second.
    - `utility/`: utility scripts.
        - `combine_chunks.py` combines chunked logs.
        - `record_limits.py` records joint limits to tweak YAML configuration files.
        - `publisher.py` is a dummy publisher.
        - `subscriber.py` is a dummy subscriber.

## Usage

- In **Full-Body Mode**, the safety layer relays a single `low_cmd` to the robot with clipping and safety checks.

    ```yaml
    topics:
      low_cmd_in: rt/safety/lowcmd_in
      low_cmd_out: rt/lowcmd
      low_state: rt/lowstate
    ```

- In **Split Mode**, the safety layer relays two `low_cmd`, one for lower-body and one for upper-body,
    with clipping and safety checks.

    ```yaml
    topics:
      low_cmd_lower_in: rt/safety/lowcmd_lower_in
      low_cmd_upper_in: rt/safety/lowcmd_upper_in
      low_cmd_out: rt/lowcmd
      low_state: rt/lowstate
    ```

- **External E-Stop** monitoring is configured with a top-level `estop` section.
  If `triggered: true` or `plugged_in: false` is received on `estop_topic`, the safety layer enters estop.
  For simulation where hardware estop is not required, set `enabled: false`.

    ```yaml
    estop:
      enabled: true
      estop_topic: h12/estop_status_raw
      poll_hz: 500.0
    ```

### `h12_safety_layer/script/safety_layer_main.py`

- **Full-Body Mode**
    - Run safety-layer in full-body mode with default safety limits:

        ```bash
        uv run h12_safety_layer/script/safety_layer_main.py --config default_safety_full.yaml
        ```

    - Run safety-layer in full-body mode with tight safety limits:

        ```bash
        uv run h12_safety_layer/script/safety_layer_main.py --config tight_safety_full.yaml
        ```

- **Split Mode**
    - Run safety-layer in split mode with default safety limits:

        ```bash
        uv run h12_safety_layer/script/safety_layer_main.py --config default_safety_split.yaml
        ```

    - Run safety-layer in split mode with tight safety limits:

        ```bash
        uv run h12_safety_layer/script/safety_layer_main.py --config tight_safety_split.yaml
        ```

### ROS2 Entry Point: `safety_node`

- Console entry point is exposed as `safety_node` and runs `h12_safety_layer/ros2/safety_node.py`
- Build and source your ROS2 workspace first:

    ```bash
    colcon build
    source install/setup.bash
    ```

- **Full-Body Mode**
    - Run with default safety limits:

        ```bash
        ros2 run h12_safety_layer safety_node --config default_safety_full.yaml
        ```

    - Run with tight safety limits:

        ```bash
        ros2 run h12_safety_layer safety_node --config tight_safety_full.yaml
        ```

- **Split Mode**
    - Run with default safety limits:

        ```bash
        ros2 run h12_safety_layer safety_node --config default_safety_split.yaml
        ```

    - Run with tight safety limits:

        ```bash
        ros2 run h12_safety_layer safety_node --config tight_safety_split.yaml
        ```

- `--config` can be a bare config name (resolved from installed package share `config/`) or an absolute path

## Detailed Documentation

- [YAML Configuration Definition](docs/config.md)
- [Safety Layer Architecture](docs/architecture.md)
