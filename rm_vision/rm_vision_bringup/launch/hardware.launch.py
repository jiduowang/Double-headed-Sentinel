import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import launch_params, robot_state_publisher, node_params, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin, name):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    
    def get_camera_detector_container(camera_node_first, camera_node_second):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node_first,
                camera_node_second,
                ComposableNode(
                    package='armor_detector_first',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector_first',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='armor_detector_second',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector_second',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node_first = get_camera_node('hik_camera', 'hik_camera::HikCameraNode', 'hik_camera_node_first')
    hik_camera_node_second = get_camera_node('hik_camera_second', 'hik_camera::HikCameraNode', 'hik_camera_node_second')
    cam_detector = get_camera_detector_container(hik_camera_node_first, hik_camera_node_second)

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+launch_params['serial_log_level']],
    )

    

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )
    
    choose = Node(
        package='armors_choose_node',
        executable='choose_node',
        name='choose',
        output='both',
    )

    info_separate = Node(
        package='info_separate',
        executable='info_seprate_node',
        name='info_separate',
        output='both',
    )

    return LaunchDescription([
        robot_state_publisher,
        cam_detector,
        delay_serial_node,
        delay_tracker_node,
        choose,
        info_separate
    ])
