#pragma once

#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>    // std::shuffle
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

//#include <json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "face_control_interfaces/msg/smile.hpp"
#include <behaviortree_cpp_v3/action_node.h>

//sing json = nlohmann::json;
using namespace std;

const std::string pose_json_file = "./robot_says_poses.json";

// Find a better place to create this singleton object...

class RobotSaysGame
{
private:
	enum Pose {
		OnHip,
		ArmOut,
		ArmToSide,
		TShoulder,	// touching
		TStomach,	// touching
		AHead,		// above
		OHead,		// on
		TNeck,		// touching
	};

	enum Sides {
		SideAny = 0,	// Don't specify left or right, allow either
		Left,			// Specify left only
		Right,			// Specify right only
		LeftAndRight	// Specify left or right randomly
	};

public:
	RobotSaysGame():
		level_start_(0),
		level_end_(0)
	{
		speech_[OnHip] = "put your * hand on your hip";
		speech_[ArmOut] = "stretch your * arm out";
		speech_[ArmToSide] = "put your * arm to your side";
		speech_[TShoulder] = "touch your shoulder with your * hand";
		speech_[TStomach] = "put your * hand on your stomach";
		speech_[AHead] = "put your * hand high above your head";
		speech_[OHead] = "put your * hand on your head";
		speech_[TNeck] = "put your * hand on your neck";

		name_[OnHip] = "OnHip";
		name_[ArmOut] = "ArmOut";
		name_[ArmToSide] = "ArmToSide";
		name_[TShoulder] = "TouchingShoulder";
		name_[TStomach] = "TouchingStomach";
		name_[AHead] = "Abovehead";
		name_[OHead] = "OnHead";
		name_[TNeck] = "TouchingNeck";

		poses_ = {OnHip, ArmOut, ArmToSide, TShoulder, TStomach, AHead, OHead, TNeck};

		levels_ = {SideAny, Left, Right, LeftAndRight};

		Init(level_start_, level_end_);
	}

	~RobotSaysGame()
	{
	}

	int NextPass(bool reset)
	{
		if (reset) {
			level_cur_ = level_start_;
		} else if (level_cur_ < level_end_) {
			level_cur_++;
		} else {
			return -1;
		}

		random_poses_ = poses_;

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

		shuffle(random_poses_.begin(), random_poses_.end(), std::default_random_engine(seed));

		cout << "Random pose list: ";
		for (auto it = random_poses_.begin(); it != random_poses_.end(); it++) {
			cout << name_[*it] << " ";
		}
		std::cout << endl;
		return 0;
	}

	// Type: any, l, r, lr
	int NextStep(string &pose_name_l, string &pose_name_r, string &pose_lr_check, string &pose_speech)
	{
		pose_name_l = "";
		pose_name_r = "";
		pose_speech = "";

		if (random_poses_.size() == 0) {
			return -1;
		}
		Pose p = random_poses_.back();
		random_poses_.pop_back();
		pose_speech = speech_[p];

		size_t marker =  pose_speech.find ('*');
		string side;
		if (marker != string::npos) {
			switch (levels_[level_cur_]) {
			default:
			case SideAny:
				side = "";
				pose_name_l = name_[p];
				pose_name_r = name_[p];
				pose_lr_check = "any";
				break;
			case Left:
				side = "left";
				pose_name_l = name_[p];
				pose_lr_check = "both";
				break;
			case Right:
				side = "right";
				pose_name_r = name_[p];
				pose_lr_check = "both";
				break;
			case LeftAndRight:
				if ((rand() % 2) == 0) {
					pose_name_l = name_[p];
					side = "left";
				} else {
					pose_name_r = name_[p];
					side = "right";
				}
				pose_lr_check = "both";
				break;
			}
			pose_speech.replace(marker, 1, side);
		}
//		cout << "NextStep: pose_name_l= " << pose_name_l << ", pose_name_r= " << pose_name_r << ", pose_speech= " << pose_speech << endl;
		return 0;
	}

	void Init(int32_t level_start, int32_t level_end)
	{
		if (level_end > GetMaxDifficulty()) {
			level_end = GetMaxDifficulty();
		}
		if (level_start > level_end) {
			level_start = 0;
		}
		level_start_ = level_start;
		level_end_ = level_end;

		NextPass(true);

		/*
		std::ifstream p(pose_json_file);
		p >> poses_;
		std::cout << poses_;
		*/
	}

	void DumpSteps()
	{
		string pose_name_l;
		string pose_name_r;
		string pose_lr_check;
		string pose_speech;

		while (!NextStep(pose_name_l, pose_name_r, pose_lr_check, pose_speech)) {
			cout << "NextStep: pose_name_l= " << pose_name_l << ", pose_name_r= " << pose_name_r << ", pose_lr_check= " << pose_lr_check << ", pose_speech= " << pose_speech << endl;
		}
		cout << endl << endl;

	}

	void TestGameData()
	{
		NextPass(true);
		DumpSteps();
		NextPass(false);
		DumpSteps();
		NextPass(false);
		DumpSteps();
		NextPass(false);
		DumpSteps();
	}

	int32_t GetMinDifficulty()
	{
		return 0;
	}

	int32_t GetMaxDifficulty()
	{
		return levels_.size() - 1;
	}

private:
	//json poses_;
	vector<enum Pose> poses_;
	vector<enum Pose> random_poses_;
	map<enum Pose, string> speech_;
	map<enum Pose, string> name_;
	vector<enum Sides> levels_;
	int32_t level_start_;
	int32_t level_end_;
	int32_t level_cur_;
};

static RobotSaysGame* game = nullptr;

class RobotSaysInitAction : public BT::SyncActionNode
{
    public:
		RobotSaysInitAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
        	return { BT::InputPort<std::string>("level_start"), BT::InputPort<std::string>("level_end") };
        }

        virtual BT::NodeStatus tick() override
        {
			if (game == nullptr) {
				game = new RobotSaysGame();
			}
        	int32_t level_start = game->GetMinDifficulty();
        	int32_t level_end = game->GetMaxDifficulty();
        	getInput<std::int32_t>("level_start", level_start);
        	getInput<std::int32_t>("level_end", level_end);

        	game->Init(level_start, level_end);
        	game->TestGameData();
            return BT::NodeStatus::SUCCESS;
        }

    private:
};

class RobotSaysNextPassAction : public BT::SyncActionNode
{
    public:
		RobotSaysNextPassAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
        	return{};
        }

        virtual BT::NodeStatus tick() override
        {
			if (game != nullptr &&
				!game->NextPass(false)) {
	            return BT::NodeStatus::SUCCESS;
			}
			return BT::NodeStatus::FAILURE;
        }

    private:
};

class RobotSaysNextStepAction : public BT::SyncActionNode
{
    public:
		RobotSaysNextStepAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<std::string>("pose_name_l"),
            		BT::OutputPort<std::string>("pose_name_r"),
					BT::OutputPort<std::string>("pose_lr_check"),
					BT::OutputPort<std::string>("pose_speech")};
        }

        virtual BT::NodeStatus tick() override
        {
    		string pose_name_l;
    		string pose_name_r;
    		string pose_lr_check;
    		string pose_speech;

    		if (game != nullptr && !game->NextStep(pose_name_l, pose_name_r, pose_lr_check, pose_speech)) {
    			cout << "NextStep: pose_name_l= " << pose_name_l << ", pose_name_r= " << pose_name_r << ", pose_lr_check= " << pose_lr_check << ", pose_speech= " << pose_speech << endl;
    			setOutput("pose_name_l", pose_name_l);
    			setOutput("pose_name_r", pose_name_r);
    			setOutput("pose_lr_check", pose_lr_check);
    			setOutput("pose_speech", pose_speech);
	            return BT::NodeStatus::SUCCESS;
    		}
			return BT::NodeStatus::FAILURE;
        }

    private:
};
