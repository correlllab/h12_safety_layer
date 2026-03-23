# YAML Configuration Definition

[Back to README](../README.md)

## Overview

The YAML configuration files define how the safety layer operates,
including DDS topic names, network settings, control parameters, safety limits, and logging behavior.
All configuration files are located in the `config/` directory.

- [Full Body Mode with Default Safety Limit](../config/default_safety_full.yaml).
- [Full Body Mode with Tight Safety Limit](../config/tight_safety_full.yaml).
- [Split Mode with Default Safety Limit](../config/default_safety_split.yaml).
- [Split Mode with Tight Safety Limit](../config/tight_safety_split.yaml).

## Configuration Settings

### `mode`

Specifies the operational mode of the safety layer.

- **Type:** string.
- **Options:**
    - `full_body_mode`: Single `low_cmd` input controlling the entire robot.
    - `split_mode`: Two separate `low_cmd` inputs for lower-body and upper-body control.
- **Example:**

    ```yaml
    mode: full_body_mode
    ```

### `topics`

Defines the DDS topic names used by the safety layer for communication.

#### Full-Body Mode Topics

- **`low_cmd_in`** (string): Input topic receiving commands to be filtered.
    - Default: `rt/safety/lowcmd_in`.
- **`low_cmd_out`** (string): Output topic where filtered commands are published.
    - Default: `rt/lowcmd`.
- **`low_state`** (string): Input topic for monitoring current robot state.
    - Default: `rt/lowstate`.

#### Split Mode Topics (in addition to `low_cmd_out` and `low_state`)

- **`low_cmd_lower_in`** (string): Input topic for lower-body commands.
    - Default: `rt/safety/lowcmd_lower_in`.
- **`low_cmd_upper_in`** (string): Input topic for upper-body commands.
    - Default: `rt/safety/lowcmd_upper_in`.

### `network`

Configures DDS Domain ID and network interface settings.

- **`domain_id`** (integer): DDS Domain ID for inter-process communication.
    - Default: `0`.
    - Must match the robot's domain ID.
- **`interface`** (string): Network interface name (empty string uses default interface).
    - Default: `''` (auto-detect).
    - Example: `eth0`, `wlan0`.

### `control`

Specifies control loop parameters.

- **`publish_hz`** (float): Frequency (Hz) at which filtered commands are published.
    - Default: `500.0`.
    - Must not exceed the safety layer's processing capability.

### `limits`

Defines safety thresholds for command clipping and emergency stop logic.
All fields under `limits` can be either:

- **Scalar**: Single value applied to all 27 joints.
- **List**: 27-entry list for per-joint customization.

#### `clip` Section

Used to constrain commands within safe operating bounds without triggering emergency stop.

- **`position_offset`** (float or list): Maximum allowed deviation from current joint position.
    - Units: radians.
    - Default: `0.05`.
    - Typical range: `0.01` to `0.1`.

- **`velocity_ratio`** (float or list): Maximum velocity as a ratio of joint's max velocity.
    - Dimensionless ratio (0.0 to 1.0).
    - Default: `0.08`.
    - Typical range: `0.05` to `0.5`.

- **`torque_ratio`** (float or list): Maximum torque as a ratio of joint's max torque.
    - Dimensionless ratio (0.0 to 1.0).
    - Default: `0.25`.
    - Typical range: `0.1` to `1.0`.

- **`kp_max`** (float or list): Maximum proportional gain for position control.
    - Units: N·m/rad.
    - Default: `500.0`.
    - Typical range: `100.0` to `500.0`.

- **`kd_max`** (float or list): Maximum derivative gain for damping.
    - Units: N·m·s/rad.
    - Default: `30.0`.
    - Typical range: `5.0` to `50.0`.

#### `estop` Section

Used for emergency stop logic. When violated, causes immediate command halt without gradual clipping.

- **`position_offset`** (float or list): Maximum position deviation for emergency stop.
    - Units: radians.
    - Default: `0.001` to `0.002` (stricter than clipping).
    - Typical range: `0.0005` to `0.01`.

- **`velocity_ratio`** (float or list): Maximum velocity ratio for emergency stop.
    - Dimensionless ratio.
    - Default varies by joint (typically `0.1` to `0.5`).
    - More lenient than clipping to allow controlled motion without triggering estop.

- **`torque_ratio`** (float or list): Maximum torque ratio for emergency stop.
    - Dimensionless ratio.
    - Default: `1.0` (no restricting, relies on other checks).
    - Typical range: `0.5` to `1.0`.

### `logging`

Controls logging behavior for recording robot states and commands.

- **`enabled`** (boolean): Whether to enable logging.
    - Default: `true`.
    - Set to `false` to disable logging for production runs.

- **`base_dir`** (string): Directory path where log chunks are stored.
    - Default: `/tmp/h12_safety_layer`.
    - Must have write permissions.
    - Example: `/home/user/h12_logs`.

- **`chunk_prefix`** (string): Prefix for chunked log filenames.
    - Default: `chunk_`.
    - Logs are named as `{chunk_prefix}{timestamp}.npz`.

## Configuration Presets

### Default Safety (`default_safety_*.yaml`)

Permissive limits suitable for normal operation with human monitoring.

- Uses scalar values for most parameters.
- Larger position offsets and velocity ratios.
- Suitable for development and testing.

### Tight Safety (`tight_safety_*.yaml`)

Strict limits with per-joint tuning for enhanced safety during autonomous operation.

- Uses per-joint lists for critical parameters (especially in `estop`).
- Tighter velocity ratios, especially for sensitive joints.
- Includes explicit limits for each joint based on its mechanical properties.

## Example Configuration (Full-Body Mode)

```yaml
mode: full_body_mode

topics:
  low_cmd_in: rt/safety/lowcmd_in
  low_cmd_out: rt/lowcmd
  low_state: rt/lowstate

network:
  domain_id: 0
  interface: ''

control:
  publish_hz: 500.0

limits:
  clip:
    position_offset: 0.05
    velocity_ratio: 0.08
    torque_ratio: 0.25
    kp_max: 500.0
    kd_max: 30.0
  estop:
    position_offset: 0.001
    velocity_ratio: 0.5
    torque_ratio: 1.0

logging:
  enabled: true
  base_dir: /tmp/h12_safety_layer
  chunk_prefix: chunk_
```

## Per-Joint Configuration Example

When using per-joint lists, the order follows the 27-DOF H1-2 joint structure:

```
Indices 0-5:   Left leg (6 joints)
Indices 6-11:  Right leg (6 joints)
Index 12:      Torso
Indices 13-19: Left arm (7 joints)
Indices 20-26: Right arm (7 joints)
```

Example of per-joint `velocity_ratio` in `estop`:

```yaml
estop:
  velocity_ratio:
    - 0.15 # left_hip_yaw_joint
    - 0.15 # left_hip_pitch_joint
    - 0.20 # left_hip_roll_joint
    # ... (continue for all 27 joints)
```

## Tips for Tuning

1. **Start Conservative**: Begin with tight safety limits and relax gradually as needed.
2. **Joint-Specific Tuning**: Different joints have different safety requirements; use per-joint lists for critical joints.
3. **Test Incrementally**: Make small changes and test thoroughly between adjustments.
4. **Monitor Logs**: Check logged data to understand clipping and estop triggers.
5. **Document Changes**: Keep notes on why specific limits were chosen for your use case.
