cmake_minimum_required(VERSION 3.8)
project(talon_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED) # allow the following two lines
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

ament_auto_find_build_dependencies()
ament_export_dependencies(rosidl_default_runtime)


# include_directories(include)

add_library(
  talon_ros2_control
  SHARED
  src/talon_hardware_interface.cpp
  include/talon_ros2_control/talon_hardware_interface.hpp
)

# Find and link phoenix/etc dependencies
set(PHOENIX_CODE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/talon_ros2_control)
include_directories(${PHOENIX_CODE_DIR})

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf")
    set(PHOENIX_LIBS_DIR ${PHOENIX_CODE_DIR}/shared_libs/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")
    set(PHOENIX_LIBS_DIR ${PHOENIX_CODE_DIR}/shared_libs/arm64)
else()
    set(PHOENIX_LIBS_DIR ${PHOENIX_CODE_DIR}/shared_libs/x86-64)
endif()

find_library(CTRE_PHOENIX NAMES CTRE_Phoenix PATHS ${PHOENIX_LIBS_DIR})
find_library(CTRE_PHOENIXCCI NAMES CTRE_PhoenixCCI PATHS ${PHOENIX_LIBS_DIR})
find_library(CTRE_PHOENIXTOOLS NAMES CTRE_PhoenixTools PATHS ${PHOENIX_LIBS_DIR})

set(SHARED_OBJECTS
    ${CTRE_PHOENIX}
    ${CTRE_PHOENIXCCI}
    ${CTRE_PHOENIXTOOLS}
)

# Link the shared object files to the target, the target is a library
target_link_libraries(talon_ros2_control PUBLIC -Wl,--no-as-needed ${SHARED_OBJECTS})

set_target_properties(talon_ros2_control PROPERTIES
    BUILD_RPATH ${PHOENIX_LIBS_DIR} # Adjust the relative path as needed
    INSTALL_RPATH ${PHOENIX_LIBS_DIR} # Used for installed binaries
    INSTALL_RPATH_USE_LINK_PATH TRUE # Include linked libraries' paths in RPATH
)

target_sources(talon_ros2_control
  PRIVATE
  ${PHOENIX_CODE_DIR}/talon_control.cpp
)

target_compile_features(talon_ros2_control PUBLIC cxx_std_20)
target_include_directories(talon_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/talon_ros2_control>
)

ament_target_dependencies(
  talon_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface talon_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/talon_ros2_control
)

install(TARGETS talon_ros2_control
  EXPORT export_talon_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_talon_ros2_control)
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
