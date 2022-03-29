#include "nav2_client.hpp"
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
#include "antenna_action.hpp"
#include "track_action.hpp"
#include "head_tilt_action.hpp"
#include "wakeword_detected.hpp"
#include "robot_says_init_actions.hpp"
#include "robot_says_next_pass.hpp"
#include "robot_says_next_step.hpp"
#include "get_random_selection.hpp"
#include "robot_seek_init_action.hpp"
#include "robot_seek_next_pose.hpp"
#include "robot_spin.hpp"
#include "object_detection_action.hpp"
#include "object_tracker_location_status_action.hpp"
#include "object_tracker_status_action.hpp"
#if defined(USE_BT_COROUTINES)
#include "scan_wait_action.hpp"
#endif
#include "save_image.hpp"
#include "robot_find_init_action.hpp"
#include "robot_find_next_step.hpp"
#include "robot_find_check_step.hpp"
#include "robot_find_set_position.hpp"

#include "detection_processor_container.hpp"
#include "detection_processor_create_action.hpp"
#include "detection_configure_action.hpp"
#include "detection_command_action.hpp"
#include "detection_select_action.hpp"
#include "detection_get_position_action.hpp"

#include "ui_input_action.hpp"
#include "log_action.hpp"
#include "move_to_object_action.hpp"

#include "pick_object_action.hpp"
#include "pick_object_test1_action.hpp"
#include "arm_goto_named_position_action.hpp"
#include "set_arm_position_action.hpp"
#include "set_gripper_position_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "transform_helper.hpp"
#include "robot_status.hpp"
#include "ui_topics.hpp"
#include "ros_common.hpp"
#include "game_settings.hpp"


#define DEFAULT_BT_XML ""

#define GAME_SETTINGS_FILENAME	"game_settings.json"

using namespace BT;

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    // Needed for passing-thru move group related parameters
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto nh = rclcpp::Node::make_shared("robot_bt", node_options);
 
    nh->declare_parameter<std::string>("bt_settings", "");

    rclcpp::Parameter bt_xml_param;
    nh->get_parameter_or("bt_xml", bt_xml_param, rclcpp::Parameter("bt_xml", DEFAULT_BT_XML));

    std::string bt_xml = bt_xml_param.value_to_string();
    RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());

    std::string bt_settings;
    nh->get_parameter("bt_settings", bt_settings);
    RCLCPP_INFO(nh->get_logger(), "Settings : %s", bt_settings.c_str());

    auto & settings = GameSettings::getInstance(GAME_SETTINGS_FILENAME);
    settings.Set(bt_settings);

#if 0 // Not working
    // Subscribe to changes in the settings parameter
    auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(nh);

	auto cb1 = [&nh, &settings](const rclcpp::Parameter & p) {

	  cout << "got callback" << std::endl;

	  RCLCPP_INFO(
		nh->get_logger(),
		"cb1: Received an update to parameter [%s]: [%s]",
		p.get_name().c_str(),
		p.as_string().c_str());

	  	settings.Set(p.as_string());
	};
	auto handle1 = param_subscriber->add_parameter_callback("bt_settings", cb1);
#endif

    // ROS common helpers and expose node 
    ROSCommon::Create(nh);

    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    factory.registerNodeType<Nav2Client>("Nav2Client");
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
    factory.registerNodeType<RobotSpin>("RobotSpin");
#if defined(USE_BT_COROUTINES)
    factory.registerNodeType<ScanWaitAction>("ScanWaitAction");
#endif
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

    factory.registerNodeType<PickObjectAction>("PickObjectAction");
    factory.registerNodeType<PickObjectTest1Action>("PickObjectTest1Action");
    factory.registerNodeType<ArmGotoNamedPositionAction>("ArmGotoNamedPositionAction");
    factory.registerNodeType<SetArmPositionAction>("SetArmPositionAction");
    factory.registerNodeType<SetGripperPositionAction>("SetGripperPositionAction");

    // Scratching your head because your new action isn't working?
    // Check the template type above since you probably copy and pasted and forgot to change both!!!!

    // The follow are node builders for those actions that need the ROS node pointer
    // FIX - use new schem that uses a common singleton for accessing the ROS node point...
    // Why didn't I use that from the get go???
    NodeBuilder builder_AntennaAction =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<AntennaAction>(name, config, nh);
    };
    factory.registerBuilder<AntennaAction>( "AntennaAction", builder_AntennaAction);

    NodeBuilder builder_TrackAction =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<TrackAction>(name, config, nh);
    };
    factory.registerBuilder<TrackAction>( "TrackAction", builder_TrackAction);

    NodeBuilder builder_HeadTiltAction =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<HeadTiltAction>(name, config, nh);
    };
    factory.registerBuilder<HeadTiltAction>( "HeadTiltAction", builder_HeadTiltAction);

    NodeBuilder builder_SmileAction =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<SmileAction>(name, config, nh);
    };
    factory.registerBuilder<SmileAction>( "SmileAction", builder_SmileAction);

    NodeBuilder builder_ObjectDetectionAction =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<ObjectDetectionAction>(name, config, nh);
    };
    factory.registerBuilder<ObjectDetectionAction>( "ObjectDetectionAction", builder_ObjectDetectionAction);

    NodeBuilder builder_PoseDetectionControlAction =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<PoseDetectionControlAction>(name, config, nh);
    };
    factory.registerBuilder<PoseDetectionControlAction>( "PoseDetectionControlAction", builder_PoseDetectionControlAction);

    NodeBuilder builder_SpeechToTextActionClient =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<SpeechToTextActionClient>(name, config, nh);
    };
    factory.registerBuilder<SpeechToTextActionClient>( "SpeechToTextActionClient", builder_SpeechToTextActionClient);

    NodeBuilder builder_TextToSpeechActionClient =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<TextToSpeechActionClient>(name, config, nh);
    };
    factory.registerBuilder<TextToSpeechActionClient>( "TextToSpeechActionClient", builder_TextToSpeechActionClient);

    NodeBuilder builder_HumanPoseDetect =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<HumanPoseDetect>(name, config, nh);
    };
    factory.registerBuilder<HumanPoseDetect>( "HumanPoseDetect", builder_HumanPoseDetect);

    NodeBuilder builder_ObjectTrackerStatusAction =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<ObjectTrackerStatusAction>(name, config, nh);
    };
    factory.registerBuilder<ObjectTrackerStatusAction>( "ObjectTrackerStatusAction", builder_ObjectTrackerStatusAction);

    NodeBuilder builder_SaveImageAction =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<SaveImageAction>(name, config, nh);
    };
    factory.registerBuilder<SaveImageAction>( "SaveImageAction", builder_SaveImageAction);

    // FIX - fold into the detection_processor functionality
    NodeBuilder builder_ObjectTrackerLocationStatusAction =
       [nh](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<ObjectTrackerLocationStatusAction>(name, config, nh);
    };
    factory.registerBuilder<ObjectTrackerLocationStatusAction>( "ObjectTrackerLocationStatusAction", builder_ObjectTrackerLocationStatusAction);

    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromFile(bt_xml);

    // Create loggers
    StdCoutLogger logger_cout(tree);
    //PublisherZMQ publisher_zmq(tree);
    FileLogger logger_file(tree, "bt_trace.fbl");

    NodeStatus status = NodeStatus::RUNNING;

    // Create transform helper singleton. Used by various nodes for coordinate transformation
    // between robot frames
    TransformHelper::Instance(nh);

    // Create object detector processor container singleton
    ObjDetProcContainer::Create(nh);

    // Create object for collecting robot status
    RobotStatus::Create(nh);

    // Create object for receiving UI topic messages
    UITopics::Create(nh);

    // code that could cause exception
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
    	rclcpp::spin_some(nh);
    	status = tree.tickRoot();

    	// Workaround - the parameter callback above does't appear to work...?
    	std::string param;
    	nh->get_parameter("bt_settings", param);
    	if (param != bt_settings) {
    		bt_settings = param;
    		settings.Set(bt_settings);
    	}

        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
