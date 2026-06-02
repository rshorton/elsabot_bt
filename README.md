# ElsaBot BT

The ElsaBot project was developed for the OpenCV AI Competition 2021 by Team GrandPlay.  Here are links to the final report and video made for the competition:
 
https://github.com/rshorton/elsabot

https://youtu.be/WZ-JJvT5fn8
 
The report provides an overview of the ElsaBot project which can help in understanding how the various ElsaBot-related repositories of this site fit together.  A block diagram is included which shows how the various ROS2 nodes that make up Elsabot fit together.
  
This ROS 2 package implements multiple nodes that implement the ElsaBot behavior tree engine and game behavior trees.  The engine and trees control the overall behavior of the robot.  You can create new trees to implement new games or functionality for the robot.

The trees are stored in the **bt_xml** directory. The provided trees include the main game tree called **bt_game_top.xml**.  This tree implements all of the trees developed for the OpenCV AI competition.  Other trees are for exercising various action nodes.  

The default xml tree is specified by the launch file **elsabot_bt.launch**.

The tree bt_ball_picker2.xml was used a while back for picking-up balls using a robot arm:
  
  * Video: https://youtu.be/iXQdU-qKR5s

The launch file for this package is used to start all of the robot-base-agnostic nodes that control the higher-level functions. The packages used for controlling the base (including navigation startup) should be started before this package.

Run this package using:

```
export DISPLAY=:0;ros2 launch elsabot_bt elsabot_bt.launch.py
```
The DISPLAY export is needed if that command is used with an ssh shell since the **viewer** node needs to open a window for displaying the camera output.

Use the elsabot_docker repo for building a suitable run-time environment for this package.

## Recent Updates

### (WIP) Integration of LLM for high-level control

A new behavior tree node called **AIAction** supports using an LLM for chat and control of the robot.  See the **bt_test_ai_chat.xml** tree for an example of using it.  That tree implements chat functionality and also supports a few tool calls from the model including:
* Get time and date
* Get camera frame and perform VLM analysis
* Get the current position on the Nav2 map
* Move the robot to a specified position using the Nav2 stack
* Get known Nav2 map locations
* Move the robot a specified distance using a relative heading
* Spin in place
* Speak (for speaking prior to the final response which is always spoken)
* Ask a user a question and get the reply
* Delay
* Set parallel tool behavior
* Get the list of currently detected objects
* Wait for a specify object type is detected
* Set reasoning mode (enables/disable LLM reasoning/thinking mode)

Each tool call is implemented using a subtree.  The ToolCallRunnerAction node processes the tool calls requested by the model.  The runner node can handle both single tool calls and parallel tool calls where more than one tool is specified to be run at the same time. It runs each tool by creating a subtree on the fly and then spinning each tree when the runner node is ticked.  The model can specify whether parallel calls should run to completion, or whether unfinished calls should be pre-empted after the first tool finishes.  (Such as move until a specified object type is detected.)

The Gemma 4 26B model is currently being used.  So far, it has been found to work extremely well.  When reasoning mode is enabled, it is more accurate but quite a bit slower when the prompt is complex.  As such, that mode can be enabled/disabled via a tool call (ie. asking the model to enable or disable it).  It is generally better to disable reasoning mode while casual chatting to reduce latency.  Also, streaming output from the model can be enabled to reduce chat-mode latency.  Currently it is disabled since there have been a few cases where the tool calls where not segmented correctly when streaming (seems to mis-handle negative numbers in the 'arguments' of the call).

Currenly, VLM processing is done using a tool call that grabs a camera frame and then uses a different session with Gemma 4 to analyze the frame.  That is done since it seems that including the images in the context reduces the accuracy of the tool calls.

The AIAction BT node uses the AISession c++ class to interact with an LLM using the Openai API (via HTTP).  The LLM is hosted on the device using vLLM which runs in another Docker container.  That container uses an Nvidia-provided image for vLLM with Gemma4 support.   See the run_primary_llm.sh script of the jetson_support repo for the command to launch vllm with gemma 4.  You could just as easily use a cloud-hosted model or one running on another local computer.  You will need to revise AIAction to use the correct host IP and port for that case (as well as credentials if needed).

See example videos:

  * https://youtube.com/shorts/LRyp4u0X1vA
  * https://youtube.com/shorts/3bK86ZycUxE



## Credits

The bt_ros2 package from Adlink-ROS (https://github.com/Adlink-ROS/BT_ros2.git) was used as the starting point a long time ago.

