# arm_2dof_wrist_py

Minimal Python replacement for the C++
`Manual2DOFWristJointByJointController` (`arm_controllers` package).

This node bypasses `ros2_control`. It subscribes to `/joy` directly and writes
SMC `SPEED_CONTROL` (`0xA2`) frames to socketcan for the two wrist joints.

> :warning: Do **not** run this node concurrently with `smc_ros2_control` on
> the same joints — they will fight on the bus.

## Mapping (matches the C++ controller)

- Pitch velocity `= axes[pitch_axis_index] * buttons[gate_button_index] * joint_max_velocities[0]`
- Roll velocity `= axes[roll_axis_index] * buttons[gate_button_index] * joint_max_velocities[1]`

The slow-mode service halves both outputs (configurable via `slow_factor`).

## Topics & services

| Type | Name | Direction |
| --- | --- | --- |
| `sensor_msgs/Joy` | `joy` | sub |
| `std_srvs/SetBool` | `~/set_slow_control_mode` | service |

## CAN behavior

- Opens socketcan on `can_interface` (default `can0`).
- On startup: sends `MOTOR_RUNNING_CMD` (`0x88`) to both motors.
- On shutdown: sends `MOTOR_STOP_CMD` (`0x81`) and `MOTOR_SHUTDOWN_CMD` (`0x80`).
- Every `1 / publish_rate_hz` seconds: emits a `0xA2` SPEED control frame per
  joint with the int32 little-endian motor count payload in bytes 4..7. Frames
  are skipped if the computed counts haven't changed since the last send.
- If no `Joy` message arrives within `joy_timeout_sec`, commands zero velocity.

## Run

```bash
ros2 launch arm_2dof_wrist_py manual_2dof_wrist.launch.py
```

Edit `config/manual_2dof_wrist_node.yaml` to change CAN node IDs, gear ratios,
joint orientations, max velocities, or joystick mapping.
