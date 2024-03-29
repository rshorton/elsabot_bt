cmake_minimum_required(VERSION 3.5)
project(elsabot_bt)

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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(speech_action_interfaces REQUIRED)
find_package(robot_head_interfaces REQUIRED)
find_package(robot_ui_interfaces REQUIRED)
find_package(object_detection_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_localization REQUIRED)

#find_package(moveit_ros_planning REQUIRED)
#find_package(moveit_ros_planning_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  behaviortree_cpp_v3
  speech_action_interfaces
  robot_head_interfaces
  robot_ui_interfaces
  object_detection_msgs
  nav2_util
  nav_msgs
  visualization_msgs
  robot_localization
  #moveit_ros_planning
  #moveit_ros_planning_interface
)

add_executable(elsabot_bt
  src/bt_ros2.cpp
  src/robot_says_game.cpp
  src/robot_seek_game.cpp
  src/robot_find_game.cpp
  src/robot_cat_game.cpp
  src/nav2_compute_path_client_util.cpp
  src/game_settings.cpp
  src/detection_processor.cpp
  src/detection_processor_container.cpp
  src/transform_helper.cpp
  src/robot_status.cpp
  src/ui_topics.cpp
  src/imu_topic.cpp
  src/ros_common.cpp
  src/nav_utils.cpp
)
ament_target_dependencies(elsabot_bt
	${dependencies}
)

include_directories(./thirdparty/nlohmann-json)

install (TARGETS
  elsabot_bt
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
