# Elsabot BT

ElsaBot behavior tree engine and game behavior trees.  The engine and trees control the overall behavior of the robot.  Create new trees to implement new games or 'jobs' for the robot.

The trees are stored in the **bt_xml** directory. The provided trees include the main game tree called **bt_game_top.xml**.  This tree implements all of the trees developed for the OpenCV AI competition.  Other trees are for exercising various action nodes.

The default xml tree is specified by the launch file **elsabot_bt.launch**.

The bt_ros2 package from Adlink-ROS (https://github.com/Adlink-ROS/BT_ros2.git) was used a starting point.

