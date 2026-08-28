# UMD Loop's General software integration

The follow subsystem is shared across all actual robotic subsystems

## Current package list in general:

<ul>
  <li><b>general_controllers</b>: Contains custom controllers/broadcasters used across all subsystems</li>
</ul>

---

## Motor Status Controller

The motor status controller is intended to both allow for state interfaces that are not typically published by the joint state broadcaster (position, velocity, and effort) to be published on a separate topic.

```bash
ros2 topic echo /<motor_status_controller_name>/motor_status
```

The controller also has some more complex functionality in that the user can directly make requests to the hardware to obtain status information about a particular piece of hardware. When prompting a `status request` for the RMDs for example, these commands would be called with the intention of a response frame which would be read in the RMD hardware interface.

```bash
  static constexpr std::array<StatusCommands, 4> kStatusCommands = {
    StatusCommands::READ_PID_CMD,
    StatusCommands::READ_ACCELERATION_CMD,
    StatusCommands::MOTOR_STATUS_1_CMD,
    StatusCommands::MOTOR_STATUS_2_CMD
  };
```

Once the request is sent, the motor status controller would publish the updated motor status. This request functionality allows status information for hardware to be updated on an as-needed basis rather than a constant stream which would overflow the CANBus. Here is an example command with drive:

```bash
ros2 service call /motor_status_controller/status_request msgs/srv/StatusReq "{joint_name: propulsion_fl_joint, request_rate: 0}"
```

Additionally, the user can make `maintenance requests` as well. These are intended to be quick requests for particular information from the hardware such as obtaining PID parameters. This is an example for the SMC motors used in the arm in particular.

`request_rate`:

```

-1: once

0: stop

> 0: send command at request_rate Hz
```

`data`:  CAN Frame data
```
i32_data: 32-bit data

i16_data: 16-bit data

u8_data: 8-bit data
```

For these fields specifically, the user must be familiar with the CAN protocol for the specific piece of hardware. As long as the data inserted (like multiple different 8 bit data fields) are kept in order, the hardware interface will sort out the proper order for the frame as well as do the endianess and hex conversions.

```bash
ros2 service call /motor_status_controller/maintenance_request msgs/srv/MaintenanceReq "{
  joint_name: wrist_roll,
  request_rate: -1,
  command_id: 0x40,
  i32_data: [],
  i16_data: [],
  u8_data: [150]
}"
```

Expected output:

```Shell
  vcan0       145   [8]  40 96 00 00 00 00 00 00
```

Science examples:

```bash
ros2 service call /motor_status_controller/status_request msgs/srv/StatusReq "{joint_name: stepper_motor_a, request_rate: -1}"
```

```bash
ros2 service call /motor_status_controller/status_request msgs/srv/StatusReq "{joint_name: dc_auger, request_rate: -1}"
```

```bash
ros2 service call /motor_status_controller/maintenance_request msgs/srv/MaintenanceReq "{
  joint_name: stepper_motor_a,
  request_rate: 1,
  command_id: 0x60,
  i32_data: [],
  i16_data: [],
  u8_data: [1]
}"
```

```bash
ros2 service call /motor_status_controller/maintenance_request msgs/srv/MaintenanceReq "{
  joint_name: dc_auger,
  request_rate: 1,
  command_id: 0x10,
  i32_data: [],
  i16_data: [],
  u8_data: [2]
}"
```

Drive examples:

```bash
ros2 service call /motor_status_controller/maintenance_request msgs/srv/MaintenanceReq "{
  joint_name: propulsion_fl_joint,
  request_rate: -1,
  command_id: 0x81,
  i32_data: [],
  i16_data: [],
  u8_data: []
}"
```

## Hardware functions

`format_status_command`: Function in the hardware interfaces that creates the status request CAN frames

`format_maintenance_command`: Function in the hardware interfaces that creates the status request CAN frames

## Available status commands

### RMD

```bash
READ_PID_CMD (0x30)
READ_ACCELERATION_CMD (0x42)
MOTOR_STATUS_1_CMD (0x9A)
MOTOR_STATUS_2_CMD (0x9C)
```

### SMC

```bash
READ_ACCELERATION_CMD (0x33)
READ_ENCODER_CMD (0x90)
READ_ABS_ANGLE_CMD (0x92)
MOTOR_STATUS_1_CMD (0x9A)
MOTOR_STATUS_2_CMD (0x9C)
```

### Stepper

The command byte is the base command ID plus the motor's node ID.

```bash
MOTOR_STATE (0x40 + node ID)
MOTOR_STATUS (0x50 + node ID)
```

### Servo

The command byte is the base command ID plus the motor's node ID.

```bash
MOTOR_STATE (0x40 + node ID)
MOTOR_STATUS (0x50 + node ID)
```

### DC

The command byte is the base command ID plus the motor's node ID.

```bash
MOTOR_STATE (0x40 + node ID)
MOTOR_STATUS (0x50 + node ID)
```

## Available maintenance commands

### RMD

```bash
WRITE_PID_TO_RAM_CMD (0x31)
u8_data:
- Current Loop P gain
- Current Loop I gain
- Speed Loop P gain
- Speed Loop I gain
- Position Loop P gain
- Position Loop I gain
i16_data: []
i32_data: []
```

```bash
WRITE_PID_TO_ROM_CMD (0x32)
u8_data:
- Current Loop P gain
- Current Loop I gain
- Speed Loop P gain
- Speed Loop I gain
- Position Loop P gain
- Position Loop I gain
i16_data: []
i32_data: []
```

```bash
WRITE_ACCELERATION_CMD (0x43)
u8_data:
- Function index
i16_data: []
i32_data:
- Acceleration
```

```bash
WRITE_ENCODER_MULTI_TURN_ZERO_CMD (0x63)
u8_data: []
i16_data: []
i32_data:
- Encoder offset
```

```bash
WRITE_CURRENT_MULTI_TURN_POS_ZERO_CMD (0x64)
u8_data: []
i16_data: []
i32_data: []
```

```bash
SYSTEM_RESET_CMD (0x76)
u8_data: []
i16_data: []
i32_data: []
```

```bash
BRAKE_RELEASE_CMD (0x77)
u8_data: []
i16_data: []
i32_data: []
```

```bash
BRAKE_LOCK_CMD (0x78)
u8_data: []
i16_data: []
i32_data: []
```

```bash
MOTOR_SHUTDOWN_CMD (0x80)
u8_data: []
i16_data: []
i32_data: []
```

```bash
MOTOR_STOP_CMD (0x81)
u8_data: []
i16_data: []
i32_data: []
```

### SMC

```bash
WRITE_ACCELERATION_CMD (0x34)
u8_data: []
i16_data: []
i32_data:
- Acceleration
```

```bash
READ_SETTINGS_CMD (0x40)
u8_data:
- Parameter ID
i16_data: []
i32_data: []
```

```bash
WRITE_SETTINGS_TO_RAM_CMD (0x42)
u8_data:
- Parameter ID
i16_data: []
i32_data: []
```

```bash
WRITE_SETTINGS_TO_ROM_CMD (0x44)
u8_data:
- Parameter ID
i16_data: []
i32_data: []
```

```bash
MOTOR_SHUTDOWN_CMD (0x80)
u8_data: []
i16_data: []
i32_data: []
```

```bash
MOTOR_STOP_CMD (0x81)
u8_data: []
i16_data: []
i32_data: []
```

```bash
MOTOR_RUNNING_CMD (0x88)
u8_data: []
i16_data: []
i32_data: []
```

```bash
CLEAR_MOTOR_ANGLE_CMD (0x95)
u8_data: []
i16_data: []
i32_data: []
```

```bash
CLEAR_ERROR_CMD (0x9B)
u8_data: []
i16_data: []
i32_data: []
```

### Stepper

```bash
PCB_HEARTBEAT_CMD (0x10)
u8_data:
- Heartbeat value
i16_data: []
i32_data: []
```

```bash
MAINTENANCE_CMD (0x60)
u8_data:
- Maintenance command option:
  - SET_CURRENT_MULTI_TURN_POS_ZERO_TO_ROM_CMD (0x00)
  - MOTOR_STOP_CMD (0x01)
  - MOTOR_SHUTDOWN_CMD (0x02)
  - CLEAR_ERRORS_CMD (0x03)
i16_data: []
i32_data: []
```

```bash
STEPPER_SPECS_CMD (0x70)
u8_data:
- One placeholder byte (required by the formatter; value is ignored)
i16_data: []
i32_data: []
```

### Servo

```bash
PCB_HEARTBEAT_CMD (0x10)
u8_data:
- Heartbeat value
i16_data: []
i32_data: []
```

```bash
MAINTENANCE_CMD (0x60)
u8_data:
- Maintenance command option:
  - SET_CURRENT_MULTI_TURN_POS_ZERO_TO_ROM_CMD (0x00)
  - MOTOR_STOP_CMD (0x01)
  - MOTOR_SHUTDOWN_CMD (0x02)
  - CLEAR_ERRORS_CMD (0x03)
i16_data: []
i32_data: []
```

```bash
SERVO_SPECS_CMD (0x70)
u8_data:
- One placeholder byte (required by the formatter; value is ignored)
i16_data: []
i32_data: []
```

### DC

```bash
PCB_HEARTBEAT_CMD (0x10)
u8_data:
- Heartbeat value
i16_data: []
i32_data: []
```

```bash
MAINTENANCE_CMD (0x60)
u8_data:
- Maintenance command option:
  - SET_CURRENT_MULTI_TURN_POS_ZERO_TO_ROM_CMD (0x00)
  - REQUEST_VECTORS_CMD (0x01)
  - MOTOR_STOP_CMD (0x02)
  - MOTOR_SHUTDOWN_CMD (0x03)
  - CLEAR_ERRORS_CMD (0x04)
i16_data: []
i32_data: []
```

```bash
DC_SPECS_CMD (0x70)
u8_data:
- One placeholder byte (required by the formatter; value is ignored)
i16_data: []
i32_data: []
```
