from http.server import executable
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()

    obs_detect_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'obs_detect.yaml'
    )
    pure_pursuit_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'pure_pursuit.yaml'
    )
    gap_follow_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'gap_follow.yaml'
    )

    # Pure pursuit
    pure_pursuit_node = Node(
        package = 'pure_pursuit',
        executable = 'pure_pursuit',
        name = 'pure_pursuit',
        parameters = [pure_pursuit_config]
    )

    # Obstacle detection
    obs_detect_node = Node(
        package = 'obs_detect',
        executable = 'obs_detect',
        name = 'obs_detect',
        parameters = [obs_detect_config]
    )

    # Obstacle avoidance
    gap_follow_node = Node(
        package = 'gap_follow',
        executable = 'gap_follow',
        name = 'gap_follow',
        parameters = [gap_follow_config]
    )


    ld.add_action(pure_pursuit_node)
    ld.add_action(obs_detect_node)
    ld.add_action(gap_follow_node)



    return ld
