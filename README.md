# h12_safety_layer

Safety layer for H1-2 that filters commands, monitors joint states, and enforces clipping and active-stop logic before forwarding to Unitree topics.

## Installation

- This repo depends on [Unitree Python SDK](https://github.com/unitreerobotics/unitree_sdk2_python) to communicate with the robot.
- `uv` Installation:
    - Easiest way to run scripts in this repo is to use [`uv`](https://docs.astral.sh/uv/getting-started/installation/)
    - Commands:

        ```bash
        uv sync # install dependencies to this repo including unitree sdk
        uv run python PATH_TO_SCRIPT
        ```

- Manual Installation:
    - `requirements.txt` lists dependencies that can be installed by `pip`.
    - Unitree SDK needs to be cloned and installed manually.
    - Commands:

        ```bash
        pip install -e . # install dependencies for this repo
        # clone unitree sdk
        cd PATH_TO_UNITREE_SDK
        pip install -e . # install unitree sdk
        ```
