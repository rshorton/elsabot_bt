import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Path
    bt_xml_dir = os.path.join(get_package_share_directory('elsabot_bt'), 'bt_xml')

    # The xArm URDF, SRDF and kinematics configs are need by action nodes using move group functionality
    moveit_config = MoveItConfigsBuilder("xarm").to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        SetParameter(name='use_sim_time', value=LaunchConfiguration("use_sim_time")),

        DeclareLaunchArgument(
            name='bt_xml',
            default_value=bt_xml_dir+'/bt_game_top.xml'),

        DeclareLaunchArgument(
            'run_bt',
            default_value='True',
            description='Whether to run the behavior tree.  Use False when developing/testing a tree'
        ),

        DeclareLaunchArgument(
            name='cmd_vel_topic', 
            default_value='cmd_vel',
            description='Topic to publish cmd_vel messages'
        ),        

        # Launch the robot head nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_head'), 'launch', 'robot_head.launch.py'))
        ),

        # Launch the speech input server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('speech_input_server'), 'launch', 'speech_in.launch.py'))
        ),

        # Launch the speech output server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('speech_output_server'), 'launch', 'speech_out.launch.py'))
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('run_bt')),
            package='elsabot_bt',
            executable='elsabot_bt',
            parameters=[
                {'bt_xml': LaunchConfiguration('bt_xml')},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
            output='screen',
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))]
        ),
    ])
