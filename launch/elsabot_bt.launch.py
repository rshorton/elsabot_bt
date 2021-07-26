import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.node import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # Path
    bt_xml_dir = os.path.join(get_package_share_directory('elsabot_bt'), 'bt_xml')

    # Parameters
    bt_xml = LaunchConfiguration('bt_xml', default=bt_xml_dir+'/bt_game_top.xml')

    run_bt = LaunchConfiguration('run_bt')
    declare_run_bt = DeclareLaunchArgument(
        'run_bt',
        default_value='True',
        description='Whether to run the behavior tree.  Use False when developing/testing a tree')


    # Launch the robot head nodes
    robot_head = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_head'), 'launch', 'robot_head.launch.py'))
    )

    # Utility to process object detections and publish location of object (currently only looks for
    # objects of type 'persion'
    clicked_point = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tracked_object_mapper'), 'launch', 'tracked_object_mapper.launch.py'))
    )

    # Launch the web video server for serving-up camera topics as video streams (to web pages)
    web_video_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('web_video_server'), 'launch', 'web_video_server.launch.py'))
    )

    # Launch the speech input server
    speech_in = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('speech_input_server'), 'launch', 'speech_in.launch.py'))
    )

    # Launch the speech output server
    speech_out = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('speech_output_server'), 'launch', 'speech_out.launch.py'))
    )

    behavior_tree = Node(
        condition=IfCondition(run_bt),
        package='elsabot_bt',
        executable='elsabot_bt',
        parameters=[{'bt_xml': bt_xml}],
        output='screen'
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_run_bt)

    ld.add_action(robot_head)
    ld.add_action(clicked_point)
    ld.add_action(web_video_server)
    ld.add_action(speech_in)
    ld.add_action(speech_out)
    ld.add_action(behavior_tree)
    return ld
