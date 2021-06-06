#include "nav2_client.hpp"
#include "nav2_compute_path_client.hpp"
#include "nav2_clear_local_cost_map.hpp"
#include "nav2_clear_global_cost_map.hpp"

#include "interrupt_event.hpp"
#include "snapshot_client.hpp"
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
#include "scan_wait_action.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "game_settings.hpp"

#define DEFAULT_BT_XML ""

#define GAME_SETTINGS_FILENAME	"game_settings.json"

using namespace BT;

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("robot_bt");
    nh->declare_parameter<std::string>("bt_xml", DEFAULT_BT_XML);
    nh->declare_parameter<std::string>("bt_settings", "");

    std::string bt_xml;
    nh->get_parameter("bt_xml", bt_xml);
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

    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    factory.registerNodeType<Nav2Client>("Nav2Client");
    factory.registerNodeType<TextCompareAction>("TextCompareAction");
    factory.registerNodeType<SpeechToTextActionClient>("SpeechToTextActionClient");
    factory.registerNodeType<TextToSpeechActionClient>("TextToSpeechActionClient");
    factory.registerNodeType<VoiceDetected>("VoiceDetected");
    factory.registerNodeType<HumanPoseDetect>("HumanPoseDetect");
    factory.registerNodeType<PoseDetectionControlAction>("PoseDetectionControlAction");
    factory.registerNodeType<SmileAction>("SmileAction");
    factory.registerNodeType<AntennaAction>("AntennaAction");
    factory.registerNodeType<TrackAction>("TrackAction");
    factory.registerNodeType<HeadTiltAction>("HeadTiltAction");
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
    factory.registerNodeType<ObjectDetectionAction>("ObjectDetectionAction");
    factory.registerNodeType<ScanWaitAction>("ScanWaitAction");
    // New action not working? Check the template type above since you probably copy and pasted and forgot to change both!!!!

    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromFile(bt_xml);

    // Create loggers
    StdCoutLogger logger_cout(tree);
    PublisherZMQ publisher_zmq(tree);
    FileLogger logger_file(tree, "bt_trace.fbl");

    NodeStatus status = NodeStatus::RUNNING;
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
