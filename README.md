# Elsabot BT

This ROS 2 package implements multiple nodes that implement the ElsaBot behavior tree engine and game behavior trees.  The engine and trees control the overall behavior of the robot.  Create new trees to implement new games or 'jobs' for the robot.

The trees are stored in the **bt_xml** directory. The provided trees include the main game tree called **bt_game_top.xml**.  This tree implements all of the trees developed for the OpenCV AI competition.  Other trees are for exercising various action nodes.

The default xml tree is specified by the launch file **elsabot_bt.launch**.

The launch file for this package is used to start all of the nodes that run on the RPi4 CPU A of the ElsaBot robot.  Run using:

```
export DISPLAY=:0;ros2 launch elsabot_bt elsabot_bt.launch.py
```
The DISPLAY export is needed if that command is used with an ssh shell since the **viewer** node needs to open a window for displaying the camera output.

Also, set the venv for the DepthAi python dependencies before running the above command. Example:

````
pushd ~/depthai/depthai-python/; . venv/bin/activate; popd
````

This is required for the **robot_head** package.

## Credits

The bt_ros2 package from Adlink-ROS (https://github.com/Adlink-ROS/BT_ros2.git) was used as the starting point.

