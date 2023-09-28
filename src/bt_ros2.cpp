#include "nav2_client.hpp"
#include "nav2_client_loop.hpp"
#include "nav2_compute_path_client.hpp"
#include "nav2_clear_local_cost_map.hpp"
#include "nav2_clear_global_cost_map.hpp"

#include "text_compare_action.hpp"
#include "speech_to_text_action_client.hpp"
#include "text_to_speech_action_client.hpp"
#include "human_pose_detect.hpp"
#include "human_pose_detection_control_action.hpp"
#include "voice_detected.hpp"
#include "smile_action.hpp"
#include "pointer_light_action.hpp"
#include "antenna_action.hpp"
#include "track_action.hpp"
#include "track_manual_action.hpp"
#include "head_tilt_action.hpp"
#include "wakeword_detected.hpp"
#include "robot_says_init_actions.hpp"
#include "robot_says_next_pass.hpp"
#include "robot_says_next_step.hpp"
#include "get_random_selection.hpp"
#include "robot_seek_init_action.hpp"
#include "robot_seek_next_pose.hpp"
#include "robot_seek_in_bounds_check_action.hpp"
#include "robot_spin.hpp"
#include "object_detection_action.hpp"
#include "object_tracker_location_status_action.hpp"
#include "object_tracker_status_action.hpp"
#include "publish_position_as_goal_action.hpp"
#include "scan_wait_action.hpp"
#include "save_image.hpp"
#include "robot_find_init_action.hpp"
#include "robot_find_next_step.hpp"
#include "robot_find_check_step.hpp"
#include "robot_find_set_position.hpp"
#include "robot_cat_game_next_pose_action.hpp"
#include "robot_cat_game_init_action.hpp"

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
#include "move_to_object_action.hpp"

#if defined(ROBOT_ARM_SUPPORT)
#include "pick_object_action.hpp"
#include "pick_object_test1_action.hpp"
#include "arm_goto_named_position_action.hpp"
#include "set_arm_position_action.hpp"
#include "set_gripper_position_action.hpp"
#endif

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "transform_helper.hpp"
#include "robot_status.hpp"
#include "ui_topics.hpp"
#include "ros_common.hpp"
#include "game_settings.hpp"
#include "imu_topic.hpp"
#include "get_robot_pose_action.hpp"

#define DEFAULT_BT_XML ""

#define GAME_SETTINGS_FILENAME	"game_settings.json"

using namespace BT;

#define REGISTER_BUILDER_WITH_ROS_NODE(node_type, ros_node_handle) \
    factory.registerBuilder<node_type>(#node_type, \
                           [ros_node_handle](const std::string& name, const NodeConfiguration& config) { \
                               return std::make_unique<node_type>(name, config, ros_node_handle); \
                           }) \


int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("robot_bt");
 
    nh->declare_parameter("bt_use_std_out_logger", false);
    bool use_std_out_logger = nh->get_parameter("bt_use_std_out_logger").as_bool();

    nh->declare_parameter("bt_use_zmq_publisher", false);
    bool use_zmq_publisher = nh->get_parameter("bt_use_zmq_publisher").as_bool();

    nh->declare_parameter("bt_log_file", "");
    std::string log_file = nh->get_parameter("bt_log_file").as_string();

    nh->declare_parameter("bt_xml", DEFAULT_BT_XML);
    std::string bt_xml = nh->get_parameter("bt_xml").as_string();
    RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());

    nh->declare_parameter("bt_settings", "");
    std::string bt_settings = nh->get_parameter("bt_settings").as_string();
    RCLCPP_INFO(nh->get_logger(), "Settings : %s", bt_settings.c_str());

    auto & settings = GameSettings::getInstance(GAME_SETTINGS_FILENAME);
    settings.Set(bt_settings);

    // ROS common helpers and expose node 
    ROSCommon::Create(nh);

    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    factory.registerNodeType<Nav2Client>("Nav2Client");
    factory.registerNodeType<Nav2ClientLoop>("Nav2ClientLoop");
    factory.registerNodeType<TextCompareAction>("TextCompareAction");
    factory.registerNodeType<VoiceDetected>("VoiceDetected");
    factory.registerNodeType<WakeWordDetected>("WakeWordDetected");
    factory.registerNodeType<RobotSaysInitAction>("RobotSaysInitAction");
    factory.registerNodeType<RobotSaysNextPassAction>("RobotSaysNextPassAction");
    factory.registerNodeType<RobotSaysNextStepAction>("RobotSaysNextStepAction");
    factory.registerNodeType<GetRandomSelectionAction>("GetRandomSelectionAction");
    factory.registerNodeType<Nav2ComputePathClient>("Nav2ComputePathClient");
    factory.registerNodeType<Nav2ClearLocalCostMap>("Nav2ClearLocalCostMap");
    factory.registerNodeType<Nav2ClearGlobalCostMap>("Nav2ClearGlobalCostMap");
    factory.registerNodeType<RobotSeekInitAction>("RobotSeekInitAction");
    factory.registerNodeType<RobotSeekNextSearchPose>("RobotSeekNextSearchPose");
    factory.registerNodeType<RobotSeekInBoundsCheckAction>("RobotSeekInBoundsCheckAction");
    factory.registerNodeType<RobotSpin>("RobotSpin");
    factory.registerNodeType<ScanWaitAction>("ScanWaitAction");
    factory.registerNodeType<RobotFindInitAction>("RobotFindInitAction");
    factory.registerNodeType<RobotFindNextStepAction>("RobotFindNextStepAction");
    factory.registerNodeType<RobotFindCheckStepAction>("RobotFindCheckStepAction");
    factory.registerNodeType<RobotFindSetPositionAction>("RobotFindSetPositionAction");

    factory.registerNodeType<UIInputAction>("UIInputAction");
    factory.registerNodeType<LogAction>("LogAction");
    factory.registerNodeType<MoveToObjectAction>("MoveToObjectAction");

    factory.registerNodeType<DetectionProcessorCreateAction>("DetectionProcessorCreateAction");
    factory.registerNodeType<DetectionConfigureAction>("DetectionConfigureAction");
    factory.registerNodeType<DetectionCommandAction>("DetectionCommandAction");
    factory.registerNodeType<DetectionGetPositionAction>("DetectionGetPositionAction");
    factory.registerNodeType<DetectionSelectAction>("DetectionSelectAction");

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
    factory.registerNodeType<ObjectTrackerStatusAction>("ObjectTrackerStatusAction");

    factory.registerNodeType<RobotCatGameNextPoseAction>("RobotCatGameNextPoseAction");
    factory.registerNodeType<RobotCatGameInitAction>("RobotCatGameInitAction");

    // Scratching your head because your new action isn't working?
    // Check the template type above since you probably copy and pasted and forgot to change both!!!!

    // These tree nodes use a builder that needs the ROS node
    REGISTER_BUILDER_WITH_ROS_NODE(AntennaAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(TrackAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(HeadTiltAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(SmileAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(PointerLightAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(ObjectDetectionAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(PoseDetectionControlAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(SpeechToTextActionClient, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(TextToSpeechActionClient, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(HumanPoseDetect, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(SaveImageAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(ObjectTrackerLocationStatusAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(PublishPositionAsGoalAction, nh);
    REGISTER_BUILDER_WITH_ROS_NODE(TrackManualAction, nh);

    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromFile(bt_xml);

    // Create loggers
    std::shared_ptr<StdCoutLogger> logger;
    if (use_std_out_logger) {
        logger = std::make_shared<StdCoutLogger>(tree);
    }

    std::shared_ptr<FileLogger> file_logger;
    if (log_file.size()) {
        file_logger = std::make_shared<FileLogger>(tree, log_file.c_str());
    }

    std::shared_ptr<PublisherZMQ> zmq_publisher;
    if (use_zmq_publisher) {
        zmq_publisher = std::make_shared<PublisherZMQ>(tree);
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

    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
    	status = tree.tickRoot();

        // Spin a while
        rclcpp::spin_until_future_complete(nh, std::promise<bool>().get_future(), std::chrono::milliseconds(50));
    }
    return 0;
}
