import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    # Path
    bt_xml_dir = os.path.join(get_package_share_directory('elsabot_bt'), 'bt_xml')

    # Parameters
    bt_xml = LaunchConfiguration('bt_xml', default=bt_xml_dir+'/bt_game_top.xml')

    # The xArm URDF, SRDF and kinematics configs are need by action nodes using move group functionality
    #moveit_config = MoveItConfigsBuilder("xarm").to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            name='bt_use_std_out_logger',
            default_value='false',
            description='Output tree debug to console'
        ),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        Node(
            package='elsabot_bt',
            executable='elsabot_bt',
            parameters=[
                {'bt_xml': bt_xml},
                {'bt_use_std_out_logger': LaunchConfiguration('bt_use_std_out_logger')}
                #moveit_config.robot_description,
                #moveit_config.robot_description_semantic,
                #moveit_config.robot_description_kinematics,
            ],
            output='screen'
        )
    ])
