#include "nav2_navigate_action.hpp"

#include "text_compare_action.hpp"
#include "text_extract_action.hpp"
#include "speech_to_text_action.hpp"
#include "text_to_speech_action.hpp"
#include "human_pose_detect.hpp"
#include "human_pose_detection_control_action.hpp"
#include "smile_action.hpp"
#include "pointer_light_action.hpp"
#include "antenna_action.hpp"
#include "track_action.hpp"
#include "track_manual_action.hpp"
#include "head_tilt_action.hpp"
#include "wakeword_detected_action.hpp"
#include "voice_detected_action.hpp"

#include "get_random_selection.hpp"
#include "robot_spin.hpp"
#include "object_detection_action.hpp"
#include "object_tracker_location_status_action.hpp"
#include "object_tracker_status_action.hpp"
#include "publish_position_as_goal_action.hpp"
#include "scan_wait_action.hpp"
#include "save_image.hpp"

#include "detection_processor_container.hpp"
#include "detection_processor_create_action.hpp"
#include "detection_configure_action.hpp"
#include "detection_command_action.hpp"
#include "detection_select_action.hpp"
#include "detection_get_position_action.hpp"
#include "get_movement_status_action.hpp"
#include "numeric_comparison_action.hpp"

#include "ui_input_action.hpp"
#include "log_action.hpp"

#include "bt_custom_type_helpers.hpp"

#if defined(ROBOT_ARM_SUPPORT)
#include "pick_object_action.hpp"
#include "pick_object_test1_action.hpp"
#include "arm_goto_named_position_action.hpp"
#include "set_arm_position_action.hpp"
#include "set_gripper_position_action.hpp"
#endif

#include "ai_action.hpp"
#include "cancel_audio_action.hpp"
#include "pause_audio_action.hpp"
#include "play_audio_file_action.hpp"
#include "resume_audio_action.hpp"

#include "transform_helper.hpp"
#include "robot_status.hpp"
#include "ui_topics.hpp"
#include "ros_common.hpp"
#include "imu_topic.hpp"
#include "get_robot_pose_action.hpp"

#include "get_time_now_action.hpp"
#include "time_since_action.hpp"
#include "tts_active_action.hpp"

#include "tool_call_time_action.hpp"
#include "tool_call_get_camera_frame_action.hpp"
#include "tool_call_analyze_camera_frame_action.hpp"
#include "tool_call_declare_action.hpp"
#include "tool_call_get_position_action.hpp"

#include "get_camera_frame_action.hpp"
#include "get_power_status_action.hpp"

#include "copy_json_key_value_into_bb_action.hpp"
#include "copy_bb_value_into_json_key_action.hpp"

#if defined(USE_GAME_FEATURES)
#include "game_support/game_settings.hpp"
#include "game_support/robot_says_init_actions.hpp"
#include "game_support/robot_says_next_pass.hpp"
#include "game_support/robot_says_next_step.hpp"
#include "game_support/robot_seek_init_action.hpp"
#include "game_support/robot_seek_next_pose.hpp"
#include "game_support/robot_seek_in_bounds_check_action.hpp"
#include "game_support/robot_find_init_action.hpp"
#include "game_support/robot_find_next_step.hpp"
#include "game_support/robot_find_check_step.hpp"
#include "game_support/robot_find_set_position.hpp"
#include "game_support/robot_cat_game_next_pose_action.hpp"
#include "game_support/robot_cat_game_init_action.hpp"
#endif

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include "behaviortree_ros2/bt_action_node.hpp"

#define DEFAULT_BT_XML ""

#if defined(USE_GAME_FEATURES)
const std::string GAME_SETTINGS_FILENAME = "game_settings.json";
#endif

using namespace BT;

#define REGISTER_BUILDER_WITH_ROS_NODE(node_type, ros_node_handle) \
    factory.registerBuilder<node_type>(#node_type, \
                           [ros_node_handle](const std::string& name, const NodeConfig& config) { \
                               return std::make_unique<node_type>(name, config, ros_node_handle); \
                           }) \


int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("robot_bt");
 
    nh->declare_parameter("bt_use_std_out_logger", false);
    bool use_std_out_logger = nh->get_parameter("bt_use_std_out_logger").as_bool();

    nh->declare_parameter("bt_log_file", "");
    std::string log_file = nh->get_parameter("bt_log_file").as_string();

    nh->declare_parameter("bt_xml", DEFAULT_BT_XML);
    std::string bt_xml = nh->get_parameter("bt_xml").as_string();
    RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());

#if defined(USE_GAME_FEATURES)
    nh->declare_parameter("bt_settings", "");
    std::string bt_settings = nh->get_parameter("bt_settings").as_string();
    RCLCPP_INFO(nh->get_logger(), "Settings : %s", bt_settings.c_str());

    auto & settings = GameSettings::getInstance(GAME_SETTINGS_FILENAME);
    settings.Set(bt_settings);
#endif    

    // Some generic parameters that are set in the blackboard of the main tree on init
    nh->declare_parameter("bt_tree_value_1", "");
    std::string bt_tree_value_1 = nh->get_parameter("bt_tree_value_1").as_string();

    nh->declare_parameter("bt_tree_value_2", "");
    std::string bt_tree_value_2 = nh->get_parameter("bt_tree_value_2").as_string();

    nh->declare_parameter("bt_tree_value_3", "");
    std::string bt_tree_value_3 = nh->get_parameter("bt_tree_value_3").as_string();

    nh->declare_parameter("bt_tree_value_4", "");
    std::string bt_tree_value_4 = nh->get_parameter("bt_tree_value_4").as_string();

    // ROS common helpers and expose node 
    ROSCommon::Create(nh);

    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

#if defined(USE_GAME_FEATURES)
    factory.registerNodeType<RobotSaysInitAction>("RobotSaysInitAction");
    factory.registerNodeType<RobotSaysNextPassAction>("RobotSaysNextPassAction");
    factory.registerNodeType<RobotSaysNextStepAction>("RobotSaysNextStepAction");

    factory.registerNodeType<RobotSeekInitAction>("RobotSeekInitAction");
    factory.registerNodeType<RobotSeekNextSearchPose>("RobotSeekNextSearchPose");
    factory.registerNodeType<RobotSeekInBoundsCheckAction>("RobotSeekInBoundsCheckAction");

    factory.registerNodeType<RobotFindInitAction>("RobotFindInitAction");
    factory.registerNodeType<RobotFindNextStepAction>("RobotFindNextStepAction");
    factory.registerNodeType<RobotFindCheckStepAction>("RobotFindCheckStepAction");
    factory.registerNodeType<RobotFindSetPositionAction>("RobotFindSetPositionAction");

    factory.registerNodeType<RobotCatGameNextPoseAction>("RobotCatGameNextPoseAction");
    factory.registerNodeType<RobotCatGameInitAction>("RobotCatGameInitAction");
#endif

#if defined(ROBOT_ARM_SUPPORT)
    factory.registerNodeType<PickObjectAction>("PickObjectAction");
    factory.registerNodeType<PickObjectTest1Action>("PickObjectTest1Action");
    factory.registerNodeType<ArmGotoNamedPositionAction>("ArmGotoNamedPositionAction");
    factory.registerNodeType<SetArmPositionAction>("SetArmPositionAction");
    factory.registerNodeType<SetGripperPositionAction>("SetGripperPositionAction");
    factory.registerNodeType<GetMovementStatusAction>("GetMovementStatusAction");
    factory.registerNodeType<NumericComparisonAction>("NumericComparisonAction");
#endif    

    factory.registerNodeType<GetRobotPoseAction>("GetRobotPoseAction");
    factory.registerNodeType<RobotSpin>("RobotSpin");

    factory.registerNodeType<TextCompareAction>("TextCompareAction");
    factory.registerNodeType<TextExtractAction>("TextExtractAction");

    factory.registerNodeType<GetRandomSelectionAction>("GetRandomSelectionAction");
    factory.registerNodeType<UIInputAction>("UIInputAction");
    factory.registerNodeType<LogAction>("LogAction");
    factory.registerNodeType<GetTimeNowAction>("GetTimeNowAction");
    factory.registerNodeType<TimeSinceAction>("TimeSinceAction");

    factory.registerNodeType<ScanWaitAction>("ScanWaitAction");

    factory.registerNodeType<ObjectTrackerStatusAction>("ObjectTrackerStatusAction");
    factory.registerNodeType<DetectionProcessorCreateAction>("DetectionProcessorCreateAction");
    factory.registerNodeType<DetectionConfigureAction>("DetectionConfigureAction");
    factory.registerNodeType<DetectionCommandAction>("DetectionCommandAction");
    factory.registerNodeType<DetectionGetPositionAction>("DetectionGetPositionAction");
    factory.registerNodeType<DetectionSelectAction>("DetectionSelectAction");

    factory.registerNodeType<AIAction>("AIAction");
    factory.registerNodeType<ToolCallTimeAction>("ToolCallTimeAction");
    factory.registerNodeType<ToolCallGetCameraFrameAction>("ToolCallGetCameraFrameAction");
    factory.registerNodeType<ToolCallAnalyzeCameraFrameAction>("ToolCallAnalyzeCameraFrameAction");
    factory.registerNodeType<ToolCallDeclareAction>("ToolCallDeclareAction");
    factory.registerNodeType<ToolCallGetPositionAction>("ToolCallGetPositionAction");

    factory.registerNodeType<GetPowerStatusAction>("GetPowerStatusAction");
    factory.registerNodeType<CopyJsonKeyValueIntoBBAction>("CopyJsonKeyValueIntoBBAction");
    factory.registerNodeType<CopyBBValueIntoJsonKeyValueAction>("CopyBBValueIntoJsonKeyValueAction");

    // These tree nodes use a builder that needs the ROS node
    REGISTER_BUILDER_WITH_ROS_NODE(AntennaAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(TrackAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(HeadTiltAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(SmileAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(PointerLightAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(TrackManualAction, nh);

    REGISTER_BUILDER_WITH_ROS_NODE(ObjectDetectionAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(ObjectTrackerLocationStatusAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(PublishPositionAsGoalAction, nh);

    REGISTER_BUILDER_WITH_ROS_NODE(PoseDetectionControlAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(HumanPoseDetect, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(SaveImageAction, nh);

    REGISTER_BUILDER_WITH_ROS_NODE(TTSActiveAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(WakeWordDetectedAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(VoiceDetectedAction, nh);

    REGISTER_BUILDER_WITH_ROS_NODE(GetCameraFrameAction, nh);

    // BehaviorTree.ros2 based nodes
    BT::RosNodeParams params;
    params.nh = nh;
    params.default_port_value = "navigate_to_pose";
    params.wait_for_server_timeout = std::chrono::seconds(10);
    factory.registerNodeType<Nav2NavigateAction>("Nav2NavigateAction", params);

    params.default_port_value = "cancel_audio";
    factory.registerNodeType<CancelAudioAction>("CancelAudioAction", params);

    params.default_port_value = "pause_audio";
    factory.registerNodeType<PauseAudioAction>("PauseAudioAction", params);

    params.default_port_value = "resume_audio";
    factory.registerNodeType<ResumeAudioAction>("ResumeAudioAction", params);

    params.default_port_value = "play_audio";
    factory.registerNodeType<PlayAudioFileAction>("PlayAudioFileAction", params);

    params.default_port_value = "play_tts";
    factory.registerNodeType<TextToSpeechAction>("TextToSpeechAction", params);

    params.default_port_value = "recognize";
    factory.registerNodeType<SpeechToTextAction>("SpeechToTextAction", params);

    ///////////////////////////////////////////

    RegisterCustomTypeHelpersJson();

    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromFile(bt_xml);

    // Get the root blackboard and set initial values
    auto blackboard = tree.rootBlackboard();

    blackboard->set<std::string>("def_value_1", bt_tree_value_1);
    blackboard->set<std::string>("def_value_2", bt_tree_value_2);
    blackboard->set<std::string>("def_value_3", bt_tree_value_3);
    blackboard->set<std::string>("def_value_4", bt_tree_value_4);
    
    // Create loggers
    std::shared_ptr<StdCoutLogger> logger;
    if (use_std_out_logger) {
        logger = std::make_shared<StdCoutLogger>(tree);
    }

    std::shared_ptr<FileLogger2> file_logger;
    if (log_file.size()) {
        file_logger = std::make_shared<FileLogger2>(tree, log_file.c_str());
    }

    NodeStatus status = NodeStatus::RUNNING;

    // Create transform helper singleton. Used by various nodes for coordinate transformation
    // between robot frames
    TransformHelper::Create(nh);

    // Create object detector processor container singleton
    ObjDetProcContainer::Create(nh);

    // Create object for collecting robot status
    RobotStatus::Create(nh);

    // Create object for receiving UI topic messages
    UITopics::Create(nh);

    // Create object for receiving IMU topic messages
    IMUTopic::Create(nh);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);

    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickOnce();
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();

    return 0;
}
