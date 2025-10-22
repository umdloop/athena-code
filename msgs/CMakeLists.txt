cmake_minimum_required(VERSION 3.8)
project(msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find all message files
file(GLOB MSG_FILES "msg/*.msg")
file(GLOB SRV_FILES "srv/*.srv")
file(GLOB ACTION_FILES "action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()
