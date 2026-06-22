import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Nav2 paths
    maps_dir = os.path.join(get_package_share_directory('amr2ax_nav2'), 'maps')
    param_dir = os.path.join(get_package_share_directory('amr2ax_nav2'), 'config')
    map_file = os.path.join(maps_dir, 'holcb2024cb202_edited.yaml')
    #map_file = os.path.join(maps_dir, 'cb112sihol.yaml')
    param_file = os.path.join(param_dir, 'xplorer_v2.yaml')
    apriltag_params = os.path.join(param_dir, 'apriltag_params.yaml')

    return LaunchDescription([
        # Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('amr2ax_nav2'),
                '/launch/bringup_launch.py'
            ]),
            launch_arguments={
                'map': map_file,
                'params_file': param_file,
            }.items(),
        ),

        # Docking Server - composable node in nav2_container
        LoadComposableNodes(
            target_container='nav2_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='opennav_docking',
                    plugin='opennav_docking::DockingServer',
                    name='docking_server',
                    parameters=[param_file],
                ),
            ],
        ),

        # AprilTag Detector
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            output='screen',
            parameters=[apriltag_params],
            remappings=[
                ('image_rect', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ]
        ),

        # Dock Pose Publisher
        Node(
            package='nouzen_bringup',
            executable='dock_pose_publisher.py',
            name='dock_pose_publisher',
            output='screen',
            parameters=[{
                'publish_rate': 10.0,
                'staleness_timeout': 5.0,
            }]
        ),

        # Lifecycle: configure docking_server dupa 5 secunde
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/docking_server', 'configure'],
                    output='screen'
                ),
            ]
        ),

        # Lifecycle: activate docking_server dupa 8 secunde
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/docking_server', 'activate'],
                    output='screen'
                ),
            ]
        ),
    ])