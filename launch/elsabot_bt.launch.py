import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
#from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    bt_xml_dir = os.path.join(get_package_share_directory('elsabot_bt'), 'bt_xml')

    # The xArm URDF, SRDF and kinematics configs are need by action nodes using move group functionality
    #moveit_config = MoveItConfigsBuilder("xarm").to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Enable sim time if true'
        ),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        DeclareLaunchArgument(
            name='bt_xml',
            default_value=bt_xml_dir+'/bt_game_top.xml'
        ),

        DeclareLaunchArgument(
            name='only_bt',
            default_value='False',
            description='Whether to only run the behavior tree node.'
        ),

        DeclareLaunchArgument(
            name='log_level',
            default_value='info',
            description='Log level'
        ),

        DeclareLaunchArgument(
            name='cmd_vel_topic', 
            default_value='cmd_vel',
            description='Topic to publish cmd_vel messages'
        ),        

        DeclareLaunchArgument(
            name='bt_use_std_out_logger',
            default_value='false',
            description='Output tree debug to console'
        ),

        # Launch the robot head nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_head'), 'launch', 'robot_head.launch.py')),
            launch_arguments={'cmd_vel_topic': 'cmd_vel/tracker'}.items(),
            condition=UnlessCondition(LaunchConfiguration('only_bt')),
        ),

        # Launch the speech input server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('speech_input_server'), 'launch', 'speech_in.launch.py')),
            condition=UnlessCondition(LaunchConfiguration('only_bt')),
        ),

        # Launch the speech output server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('speech_output_server'), 'launch', 'speech_out.launch.py')),
            condition=UnlessCondition(LaunchConfiguration('only_bt')),
        ),

        Node(
            package='elsabot_bt',
            executable='elsabot_bt',
            parameters=[
                {'bt_xml': LaunchConfiguration('bt_xml', default=bt_xml_dir+'/bt_game_v2.xml')},
                {'bt_use_std_out_logger': LaunchConfiguration('bt_use_std_out_logger')}
                #moveit_config.robot_description,
                #moveit_config.robot_description_semantic,
                #moveit_config.robot_description_kinematics,
            ],
            output='screen',
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
