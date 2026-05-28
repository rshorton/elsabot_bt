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

option(USE_GAME_FEATURES "Include game features" OFF)
option(USE_ROBOT_ARM_SUPPORT "Include robot arm support" OFF)

if(USE_ROBOT_ARM_SUPPORT)
    add_compile_definitions(USE_ROBOT_ARM_SUPPORT)
endif()

message(STATUS ${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/submodules/BehaviorTree.ROS2/behaviortree_ros2/include/)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(speech_action_interfaces REQUIRED)
find_package(elsabot_audio_output_interfaces REQUIRED)
find_package(robot_head_interfaces REQUIRED)
find_package(robot_ui_interfaces REQUIRED)
find_package(object_detection_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(CURL REQUIRED)

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
  behaviortree_cpp
  CURL
  cv_bridge
  elsabot_audio_output_interfaces
  geometry_msgs
  image_transport
  nav_msgs
  nav2_behavior_tree
  nav2_msgs
  nav2_util
  object_detection_msgs
  rclcpp
  rclcpp_action
  robot_head_interfaces
  robot_localization
  robot_ui_interfaces
  sensor_msgs
  speech_action_interfaces
  std_msgs
  visualization_msgs
  #moveit_ros_planning
  #moveit_ros_planning_interface
)

set(elsabot_bt_sources
  src/ai_action.cpp
  src/ai_session.cpp
  src/base64.cpp
  src/bt_ros2.cpp
  src/cancel_audio_action.cpp
  src/copy_bb_value_into_json_key_action.cpp
  src/copy_json_key_value_into_bb_action.cpp
  src/detection_get_objects_action.cpp
  src/detection_processor_container.cpp
  src/detection_processor.cpp
  src/detection_wait_for_object_detected_action.cpp
  src/http_request.cpp
  src/image_utils.cpp
  src/imu_topic.cpp
  src/log_action.cpp
  src/nav_utils.cpp
  src/nav2_navigate_action.cpp
  src/pause_audio_action.cpp
  src/play_audio_file_action.cpp
  src/resume_audio_action.cpp
  src/robot_status.cpp
  src/ros_common.cpp
  src/speech_to_text_action.cpp
  src/text_to_speech_action.cpp
  src/tool_call_data.cpp
  src/transform_helper.cpp
  src/ui_topics.cpp
)

if(USE_GAME_FEATURES)
  set(elsabot_bt_sources ${elsabot_bt_sources}
      src/game_support/game_settings.cpp
      src/game_support/robot_cat_game.cpp
      src/game_support/robot_find_game.cpp
      src/game_support/robot_says_game.cpp
      src/game_support/robot_seek_game.cpp
  )
  add_compile_definitions(USE_GAME_FEATURES)
endif()

add_executable(elsabot_bt ${elsabot_bt_sources})

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
