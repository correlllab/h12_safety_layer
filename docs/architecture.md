# Safety Layer Architecture

[Back to README](../README.md)

## Overview

The safety layer is a relay that intercepts command messages to the H1-2 robot, applies safety checks and clipping, monitors the robot's state for dangerous conditions, and forwards filtered commands to the robot. It operates in two modes: **Full-Body Mode** (single command input) and **Split Mode** (separate lower/upper body inputs).

## Core Components

### Command Inputs

- **Full-Body Mode**: one reader on the `low_cmd_in` topic.
- **Split Mode**: two readers, on `low_cmd_lower_in` and `low_cmd_upper_in`.
- Every input (commands and `low_state`) is a listener-free `LatestReader`
  (`core/latest_reader.py`) with `KEEP_LAST(1)` QoS, polled by the relay loop.
  CycloneDDS's C receive thread overwrites the cached sample and never waits
  on Python, so a slow relay cycle skips stale samples instead of queueing
  them into a standing delay.

### Safety Checks Layer

- **Clipping**: Constrains commands within safe bounds without halting.
- **Estop (Emergency Stop)**: Monitors robot state and incoming commands for dangerous conditions.
    - Triggered on: extreme position deviation, excessive velocity, unrealistic torque commands.
    - Effect: Immediately switches to safe (zero velocity) command.

### Relay Loop

- Runs at configurable frequency (default 500 Hz).
- Each cycle: check the newest `low_state`, validate and clip the newest
  command on each input, then publish the filtered command to `low_cmd_out`.
- If a cycle runs long, the publish rate drops; commands are never delayed.

### State Monitoring

- Checks the newest `low_state` each relay cycle.
- Triggers estop if state exceeds safety thresholds, or (real robot only)
  if `low_state` arrives more than `estop.state_lag_ms` late against the
  robot's `tick` clock.
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
    │  Command intake                 │
    │  (_take_commands, full input)   │
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
    │  Relay Loop         │◄───────►│  low_state      │
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
    │ Lower intake       │           │ Upper intake       │
    │ (_take_commands)   │           │ (_take_commands)   │
    │                    │           │                    │
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
    │  Relay Loop          │◄───────►│  low_state      │
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

1. Takes the newest `LowCmd` on each input in the relay loop (superseded
   commands are never validated or published)
2. Calls `clip_low_cmd()` to constrain values:
    - Position: clipped to ±`position_offset` from current state
    - Velocity: scaled to max `velocity_ratio × joint_max_velocity`
    - Torque: scaled to max `torque_ratio × joint_max_torque`
    - Gains (Kp, Kd): clamped to specified maximums
3. Stores clipped command as `_desired_cmd_*` (protected by lock)
4. Returns early if validation fails → triggers estop

### State Monitoring (Estop)

1. Takes the newest `LowState` in the relay loop
2. On DDS domain 0 (real robot) with `estop.state_lag_ms > 0`, estops if the
   sample arrived later than that against the robot's `tick` clock
   (`_update_state_lag`; the sims stamp `tick` in sim steps, so it is off there)
3. Calls `check_estop_limits()` with stricter thresholds
4. If any joint violates limits:
    - Sets `_estop = True`
    - Logs event with violation details
    - Prints debug info (position, velocity, acceleration commands vs actual)
5. The relay loop detects `_estop` and forces zero-velocity command

### Publishing

1. Every `dt = 1/publish_hz` seconds:
    - If estop: publish `make_estop_cmd()` (zero velocity)
    - Else: publish stored `_desired_cmd`
2. Calculates CRC checksum
3. Logs published command and current state

## Synchronization

The safety layer uses a single lock (`_lock`) to protect shared state:

- `_desired_cmd_full` / `_desired_cmd_lower` / `_desired_cmd_upper`.
- `_estop` and `_estop_reason`.
- `_last_cmd` and the `low_state` lag baseline.

Lock is acquired:

- In the relay loop when updating desired commands, checking state limits,
  and reading the desired command and estop state.
- In the external estop monitor thread when latching estop.

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
