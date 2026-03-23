# Safety Layer Architecture

[Back to README](../README.md)

## Overview

The safety layer is a relay that intercepts command messages to the H1-2 robot, applies safety checks and clipping, monitors the robot's state for dangerous conditions, and forwards filtered commands to the robot. It operates in two modes: **Full-Body Mode** (single command input) and **Split Mode** (separate lower/upper body inputs).

## Core Components

### Command Subscriptions

- **Full-Body Mode**: Single subscriber on `low_cmd_in` topic.
- **Split Mode**: Two subscribers on `low_cmd_lower_in` and `low_cmd_upper_in` topics.

### Safety Checks Layer

- **Clipping**: Constrains commands within safe bounds without halting.
- **Estop (Emergency Stop)**: Monitors robot state and incoming commands for dangerous conditions.
    - Triggered on: extreme position deviation, excessive velocity, unrealistic torque commands.
    - Effect: Immediately switches to safe (zero velocity) command.

### Publisher Loop

- Runs at configurable frequency (default 500 Hz).
- Publishes filtered commands to `low_cmd_out` topic.

### State Subscriber

- Monitors `low_state` topic for robot joint/IMU data.
- Triggers estop if state exceeds safety thresholds.
- Provides data for logging.

### Async Logger

- Records robot state and command data to HDF5 chunks.
- Runs in separate thread to avoid blocking control loop.
- Configurable chunk size and write frequency.

## Data Flow Diagrams

### Full-Body Mode

```
┌─────────────────┐
│  low_cmd_in     │  External command source
│    topic        │
└────────┬────────┘
         │
         ↓
    ┌─────────────────────────────────┐
    │  Command Handler (_on_low_cmd   │
    │       _full)                    │
    │                                 │
    │  • Receive command              │
    │  • Validate format              │
    │  • Apply clipping               │
    │  • Update desired_cmd_full      │
    └──────────┬──────────────────────┘
               │
               |  Clipped command
               │  stored
               │
    ┌──────────▼──────────┐         ┌─────────────────┐
    │  Publisher Loop     │◄───────►│  low_state      │
    │  (500 Hz)           │         │    topic        │
    │                     │         └────────┬────────┘
    │  • Get desired cmd  │                  │
    │  • Check estop      │         State monitoring
    │  • Publish          │         • Position limits
    │  • Update log cache │         • Velocity limits
    └──────────┬──────────┘         • Torque limits
               │
               ↓
    ┌──────────────────────┐
    │  low_cmd_out topic   │
    │                      │
    │ (to robot)           │
    └──────────────────────┘
               │
               ↓
    ┌──────────────────────-┐
    │  H1-2 Robot Controller|
    └──────────────────────-┘
```

### Split Mode

```
┌──────────────────────┐              ┌──────────────────────┐
│ low_cmd_lower_in     │              │ low_cmd_upper_in     │
│   topic              │              │   topic              │
│ (legs + lower body)  │              │ (torso + arms)       │
└──────────┬───────────┘              └──────────┬───────────┘
           │                                     │
           ↓                                     ↓
    ┌────────────────────┐           ┌────────────────────┐
    │ Lower Handler      │           │ Upper Handler      │
    │ (_on_low_cmd       │           │ (_on_low_cmd_upper)│
    │  _lower)           │           │                    │
    │                    │           │ • Validate         │
    │ • Validate         │           │ • Clip             │
    │ • Clip             │           │ • Update           │
    │ • Update           │           │   desired_cmd_upper│
    │   desired_cmd_lower│           │                    │
    └──────────┬─────────┘           └──────────-┬────────┘
               │                                 │
               │    Clipped commands             │
               │    stored separately            │
               │                                 │
               └────────────┬────────────────────┘
                            │
                            ↓
                    ┌───────────────────┐
                    │  Merge Step       │
                    │                   │
                    │ Combine lower     │
                    │ (indices 0-11)    │
                    │ with upper        │
                    │ (indices 12-26)   │
                    │ into single       │
                    │ LowCmd message    │
                    └─────────┬─────────┘
                              │
              | Merged command
              │
    ┌─────────▼────────────┐         ┌─────────────────┐
    │  Publisher Loop      │◄───────►│  low_state      │
    │  (500 Hz)            │         │    topic        │
    │                      │         └────────┬────────┘
    │  • Merge split cmds  │                  │
    │  • Check estop       │         State monitoring
    │  • Publish           │         • Position limits
    │  • Update log cache  │         • Velocity limits
    └─────────┬────────────┘         • Torque limits
              │
              ↓
    ┌──────────────────────┐
    │  low_cmd_out topic   │
    │                      │
    │ (single merged cmd   │
    │  to robot)           │
    └──────────────────────┘
              │
              ↓
    ┌──────────────────────┐
    │ H1-2 Robot Controller|
    └──────────────────────┘
```

## Safety Check Execution

### Command Validation (Clipping)

1. Receives incoming `LowCmd` message on callback thread
2. Calls `clip_low_cmd()` to constrain values:
    - Position: clipped to ±`position_offset` from current state
    - Velocity: scaled to max `velocity_ratio × joint_max_velocity`
    - Torque: scaled to max `torque_ratio × joint_max_torque`
    - Gains (Kp, Kd): clamped to specified maximums
3. Stores clipped command as `_desired_cmd_*` (protected by lock)
4. Returns early if validation fails → triggers estop

### State Monitoring (Estop)

1. Receives `LowState` message on callback thread
2. Calls `check_estop_limits()` with stricter thresholds
3. If any joint violates limits:
    - Sets `_estop = True`
    - Logs event with violation details
    - Prints debug info (position, velocity, acceleration commands vs actual)
4. Publisher loop detects `_estop` and forces zero-velocity command

### Publisher Loop

1. Every `dt = 1/publish_hz` seconds:
    - If estop: publish `make_estop_cmd()` (zero velocity)
    - Else: publish stored `_desired_cmd`
2. Calculates CRC checksum
3. Logs published command and current state

## Synchronization

The safety layer uses a single lock (`_lock`) to protect shared state:

- `_desired_cmd_full` / `_desired_cmd_lower` / `_desired_cmd_upper`.
- `_estop` and `_estop_reason`.
- `_last_cmd` and command component caches (`_last_q_cmd`, `_last_dq_cmd`, etc.).

Lock is acquired:

- In command callbacks when updating desired commands.
- In state callback when checking limits.
- In publisher loop when reading desired command and estop state.

This ensures no race conditions between state monitoring, command updates, and publishing.

## Data Logging

When enabled, the logger collects per-cycle:

- **Timestamp**: Unix time when state message arrived.
- **Motor State**: Mode, position (q), velocity (dq), acceleration (ddq), torque, temperature, voltage.
- **IMU State**: Quaternion, gyroscope, accelerometer, roll-pitch-yaw, temperature.
- **Command**: Last published q_cmd, dq_cmd, tau_cmd, kp_cmd, kd_cmd.

Data is:

- Buffered in-memory (up to `LOG_MAX_QUEUE_SIZE` samples).
- Written to disk asynchronously at `LOG_WRITE_HZ` (5 Hz default).
- Stored as npz "chunks" (e.g., `chunk_1234.npz`) containing up to `LOG_SAMPLES_PER_CHUNK` (1000) samples.
- Combined post-run using `combine_chunks.py` utility.
