# ElsaBot BT

The ElsaBot project was developed for the OpenCV AI Competition 2021 by Team GrandPlay.  Here are links to the final report and video made for the competition:
 
https://github.com/rshorton/elsabot

https://youtu.be/WZ-JJvT5fn8
 
The report provides an overview of the ElsaBot project which can help in understanding how the various ElsaBot-related repositories of this site fit together.  A block diagram is included which shows how the various ROS2 nodes that make up Elsabot fit together.
  
This ROS 2 package implements multiple nodes that implement the ElsaBot behavior tree engine and game behavior trees.  The engine and trees control the overall behavior of the robot.  Create new trees to implement new games or functionality for the robot.

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

A new behavior tree node called **AIAction** supports using an LLM for chat and control of the robot.  See the **bt_test_ai_chat.xml** tree for an example of using.  That tree implements chat functionality and also support a few tool-calls from the model including:
* Get time and date
* Get camera frame and perform VLM analysis
* Get the current position on the Nav2 map
* Move the robot to a specified position using the Nav2 stack
* Get Nav2 map locations (WIP)

Some of the tool calls are implemented using specific BT action nodes, while others are implemented by composing together existing nodes.

The Gemma 4 26B model is currently being used.  So far, it has been found to work extremely well.

See example videos:

  * https://youtube.com/shorts/LRyp4u0X1vA
  * https://youtube.com/shorts/3bK86ZycUxE

As an example of the capabilities of the Gemma 4 model, I asked Elsabot which of the known map locations she was closest.  She used a tool call to request the current position and then said she was near the robot home location.  I was curious if she guessed so I asked...

```
Me: Can you explain how you came up with that?

AI: I compared my current coordinates to the coordinates of the locations in my map. Since my current position is very close to the coordinates for "robot home," I determined that is my closest location. 🙂

Me: Could you give me the specifics of that comparison?

AI: My current position is: x: 0.36, y: -0.29

1. Robot Home (x: 0.55, y: -0.22): The distance is approximately 0.20 meters.
2. Office TV (x: -0.29, y: 0.10): The distance is approximately 0.74 meters.
3. Playroom TV (x: -4.48, y: 0.14): The distance is approximately 4.85 meters.

Since 0.20 meters is the smallest distance, I concluded I am closest to my robot home. 🙂
```


## Credits

The bt_ros2 package from Adlink-ROS (https://github.com/Adlink-ROS/BT_ros2.git) was used as the starting point.

