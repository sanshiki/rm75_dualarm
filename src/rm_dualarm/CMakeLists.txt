cmake_minimum_required(VERSION 3.8)
project(rm_dualarm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# --- C++ executable: servo_bridge ---
add_executable(servo_bridge src/servo_bridge.cpp)
ament_target_dependencies(servo_bridge
  rclcpp
  rm_ros_interfaces
  trajectory_msgs
  geometry_msgs
  std_msgs
  tf2_ros
)

# --- Python scripts ---
install(PROGRAMS
  src/mouse_teleop_node.py
  src/pose_tracking_node.py
  src/vr_teleop_node.py
  src/vr_base_broadcaster.py
  scripts/pose_init.py
  scripts/vr_calibration_capture.py
  scripts/analyze_dual_teleop_bag.py
  scripts/dual_joint_state_merger.py
  scripts/relay_receiver.py
  scripts/trajectory_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install C++ target
install(TARGETS
  servo_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
