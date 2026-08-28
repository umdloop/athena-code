# bringup

Mission-level launch files for Athena. Each file composes subsystem launches for a specific competition mission.

## Modes

| Mode             | What runs                                            |
| ---------------- | ---------------------------------------------------- |
| `jetson`       | Hardware and compute nodes — run on the rover       |
| `base_station` | Operator teleop nodes — run on the laptop           |
| `standalone`   | Everything on one machine (default, for development) |

---

## Delivery

Drive + Arm + GPS + Heading

```bash
# Rover
source install/setup.bash
ros2 launch bringup delivery.launch.py mode:=jetson

# Operator laptop
source install/setup.bash
ros2 launch bringup delivery.launch.py mode:=base_station

# Single machine
source install/setup.bash
ros2 launch bringup delivery.launch.py

# Mock hardware (no physical rover)
source install/setup.bash
ros2 launch bringup delivery.launch.py use_mock_hardware:=true use_sim:=true

# 3 DOF wrist
source install/setup.bash
ros2 launch bringup delivery.launch.py use_3dof:=true
```

### Parameters

| Parameter                | Default                       | Choices                                                                                        | Description                                                                            |
| ------------------------ | ----------------------------- | ---------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| `mode`                 | `standalone`                | `standalone` `jetson` `base_station`                                                     | Deployment target                                                                      |
| `use_sim`              | `false`                     | `true` `false`                                                                             | Launch RViz2                                                                           |
| `use_mock_hardware`    | `false`                     | `true` `false`                                                                             | Mock hardware for drive and arm                                                        |
| `mock_sensor_commands` | `false`                     | `true` `false`                                                                             | Mock sensor command interfaces (requires`use_mock_hardware`)                         |
| `robot_controller`     | `rear_ackermann_controller` | `rear_ackermann_controller` `front_ackermann_controller` `ackermann_steering_controller` | Drive controller to start                                                              |
| `use_3dof`             | `false`                     | `true` `false`                                                                             | Enable 3 DOF wrist joints on arm                                                       |
| `deactivate_talon`     | `false`                     | `true` `false`                                                                             | Deactivate talon joints in URDF (use with`use_mock_hardware` to prevent CAN traffic) |

---

## Autonomous Navigation

Drive + Nav2 + ZED + GPS + Localizer + Heading

```bash
# Rover
source install/setup.bash
ros2 launch bringup autonav.launch.py mode:=jetson

# Operator laptop
source install/setup.bash
ros2 launch bringup autonav.launch.py mode:=base_station

# Single machine
source install/setup.bash
ros2 launch bringup autonav.launch.py

# Use EKF localizer instead of ZED spatial localization
source install/setup.bash
ros2 launch bringup autonav.launch.py use_zed_localizer:=false

# Disable GNSS fusion in ZED
source install/setup.bash
ros2 launch bringup autonav.launch.py enable_gnss:=false

# Custom Nav2 params
source install/setup.bash
ros2 launch bringup autonav.launch.py params_file:=/path/to/params.yaml

# Debug nav2 verbosity
source install/setup.bash
ros2 launch bringup autonav.launch.py log_level:=debug
```

### Parameters

| Parameter                | Default                                                                           | Choices                                                                                        | Description                                                               |
| ------------------------ | --------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| `mode`                 | `standalone`                                                                    | `standalone` `jetson` `base_station`                                                     | Deployment target                                                         |
| `use_sim`              | `false`                                                                         | `true` `false`                                                                             | Enable sim time and RViz2                                                 |
| `use_mock_hardware`    | `false`                                                                         | `true` `false`                                                                             | Mock hardware for drive                                                   |
| `mock_sensor_commands` | `false`                                                                         | `true` `false`                                                                             | Mock sensor command interfaces (requires`use_mock_hardware`)            |
| `robot_controller`     | `rear_ackermann_controller`                                                     | `rear_ackermann_controller` `front_ackermann_controller` `ackermann_steering_controller` | Drive controller to start                                                 |
| `use_zed_localizer`    | `true`                                                                          | `true` `false`                                                                             | Use ZED spatial localization instead of EKF localizer                     |
| `enable_gnss`          | `true`                                                                          | `true` `false`                                                                             | Enable GNSS fusion inside the ZED camera                                  |
| `use_minimal`          | `false`                                                                         | `true` `false`                                                                             | Use minimal Nav2 config (empty costmaps, basic BT) for ackermann testing  |
| `params_file`          | `nav2_params.yaml` (or `nav2_params_minimal.yaml` when `use_minimal:=true`) | —                                                                                             | Full path to Nav2 params YAML; overrides`use_minimal` if set explicitly |
| `use_dem`              | `false`                                                                         | `true` `false`                                                                             | Enable DEM costmap layer                                                  |
| `use_respawn`          | `false`                                                                         | `true` `false`                                                                             | Respawn nav2 nodes on crash                                               |
| `log_level`            | `info`                                                                          | —                                                                                             | Log level for nav2 nodes                                                  |
| `use_config`           | `false`                                                                         | `true` `false`                                                                             | Launch the Nav2 config GUI                                                |

---

## Equipment Servicing

Drive + Arm + GPS + Heading

```bash
# Rover
source install/setup.bash
ros2 launch bringup equipment_servicing.launch.py mode:=jetson

# Operator laptop
source install/setup.bash
ros2 launch bringup equipment_servicing.launch.py mode:=base_station

# Single machine
source install/setup.bash
ros2 launch bringup equipment_servicing.launch.py

# Mock hardware (no physical rover)
source install/setup.bash
ros2 launch bringup equipment_servicing.launch.py use_mock_hardware:=true use_sim:=true

# 3 DOF wrist
source install/setup.bash
ros2 launch bringup equipment_servicing.launch.py use_3dof:=true
```

### Parameters

| Parameter                | Default                       | Choices                                                                                        | Description                                                                            |
| ------------------------ | ----------------------------- | ---------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| `mode`                 | `standalone`                | `standalone` `jetson` `base_station`                                                     | Deployment target                                                                      |
| `use_sim`              | `false`                     | `true` `false`                                                                             | Launch RViz2                                                                           |
| `use_mock_hardware`    | `false`                     | `true` `false`                                                                             | Mock hardware for drive and arm                                                        |
| `mock_sensor_commands` | `false`                     | `true` `false`                                                                             | Mock sensor command interfaces (requires`use_mock_hardware`)                         |
| `robot_controller`     | `rear_ackermann_controller` | `rear_ackermann_controller` `front_ackermann_controller` `ackermann_steering_controller` | Drive controller to start                                                              |
| `use_3dof`             | `false`                     | `true` `false`                                                                             | Enable 3 DOF wrist joints on arm                                                       |
| `deactivate_talon`     | `false`                     | `true` `false`                                                                             | Deactivate talon joints in URDF (use with`use_mock_hardware` to prevent CAN traffic) |

---

## Science

Drive + Science + GPS + Heading + LED Indicator

```bash
# Rover
source install/setup.bash
ros2 launch bringup science.launch.py mode:=jetson

# Operator laptop
source install/setup.bash
ros2 launch bringup science.launch.py mode:=base_station

# Single machine
source install/setup.bash
ros2 launch bringup science.launch.py

# Mock hardware (no physical rover)
source install/setup.bash
ros2 launch bringup science.launch.py use_mock_hardware:=true use_sim:=true

# Custom CAN interface
source install/setup.bash
ros2 launch bringup science.launch.py can_interface:=vcan0
```

### Parameters

| Parameter                | Default                       | Choices                                                                                        | Description                                                        |
| ------------------------ | ----------------------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------ |
| `mode`                 | `standalone`                | `standalone` `jetson` `base_station`                                                     | Deployment target                                                  |
| `use_sim`              | `false`                     | `true` `false`                                                                             | Launch RViz2 and use simulated GPS                                 |
| `use_mock_hardware`    | `false`                     | `true` `false`                                                                             | Mock hardware for drive and science                                |
| `mock_sensor_commands` | `false`                     | `true` `false`                                                                             | Mock sensor command interfaces (requires`use_mock_hardware`)     |
| `robot_controller`     | `rear_ackermann_controller` | `rear_ackermann_controller` `front_ackermann_controller` `ackermann_steering_controller` | Drive controller to start                                          |
| `deactivate_talon`     | `false`                     | `true` `false`                                                                             | Deactivate Talon joints in the science URDF to prevent CAN traffic |
| `can_interface`        | `can1`                      | —                                                                                             | CAN interface used by drive and science hardware                   |

---

## Node breakdown by mode

### Delivery / Equipment Servicing

| Node group                   | jetson | base_station | standalone |
| ---------------------------- | :----: | :----------: | :--------: |
| Drive hardware + controllers |   ✓   |              |     ✓     |
| Drive teleop (joystick)      |        |      ✓      |     ✓     |
| Arm hardware + controllers   |   ✓   |              |     ✓     |
| Arm teleop (joystick)        |        |      ✓      |     ✓     |
| GPS (athena_gps / pixhawk)   |   ✓   |              |     ✓     |
| Magnetometer heading         |   ✓   |              |     ✓     |

### Autonomous Navigation

| Node group                                   | jetson | base_station | standalone |
| -------------------------------------------- | :----: | :----------: | :--------: |
| Drive hardware + controllers                 |   ✓   |              |     ✓     |
| Drive teleop (joystick)                      |        |      ✓      |     ✓     |
| ZED camera                                   |   ✓   |              |     ✓     |
| GPS (athena_gps / pixhawk)                   |   ✓   |              |     ✓     |
| ZED spatial localizer (default)              |   ✓   |              |     ✓     |
| EKF localizer (`use_zed_localizer:=false`) |   ✓   |              |     ✓     |
| Nav2 stack                                   |   ✓   |              |     ✓     |
| Magnetometer heading                         |   ✓   |              |     ✓     |

### Science

| Node group                     | jetson | base_station | standalone |
| ------------------------------ | :----: | :----------: | :--------: |
| Drive hardware + controllers   |   ✓   |              |     ✓     |
| Drive teleop (joystick)        |        |      ✓      |     ✓     |
| Science hardware + controllers |   ✓   |              |     ✓     |
| Science teleop (joystick)      |        |      ✓      |     ✓     |
| GPS (athena_gps / pixhawk)     |   ✓   |              |     ✓     |
| Magnetometer heading           |   ✓   |              |     ✓     |
| LED indicator                  |   ✓   |              |     ✓     |

### Namespace CLI

#### How do use arm controller manager cli

During the delivery or equipment servicing missions, the arm controller manager is in a different namespace, so to monitor the controller and hardware status, use this formatting:

```bash
ros2 control <command> -c /arm/controller_manager
```

#### How do use science controller manager cli

During the science mission, the science controller manager is in a different namespace, so to monitor the controller and hardware status, use this formatting:

```bash
ros2 control <command> -c /science/controller_manager
```
