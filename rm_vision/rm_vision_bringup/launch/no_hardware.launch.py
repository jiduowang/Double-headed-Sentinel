import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import launch_params, robot_state_publisher, node_params, tracker_node
    from launch_ros.actions import Node
    from launch import LaunchDescription

    detector_first_node = Node(
        package='armor_detector_first',
        executable='armor_detector_first_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'armor_detector:='+launch_params['detector_log_level']],
    )

    detector_second_node = Node(
        package='armor_detector_second',
        executable='armor_detector_second_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'armor_detector:='+launch_params['detector_log_level']],
    )

    return LaunchDescription([
        robot_state_publisher,
        detector_first_node,
        detector_second_node,
        tracker_node,
    ])
