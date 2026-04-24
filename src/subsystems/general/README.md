ros2 service call /motor_status_controller/status_request msgs/srv/StatusReq "{joint_name: propulsion_fl_joint, request_rate: 0}"

ros2 service call /motor_status_controller/maintenance_request msgs/srv/MaintenanceReq "{
  joint_name: propulsion_fl_joint,
  request_rate: -1,
  command_id: 0x78,
  i32_data: [],
  i16_data: [],
  u8_data: []
}"