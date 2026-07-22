import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_vehicle_simulator_share():
    try:
        return get_package_share_directory('vehicle_simulator')
    except PackageNotFoundError:
        # Support launching directly from the source tree before installation.
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _include(package_name, launch_name, launch_arguments=None):
    return IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', launch_name)
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    camera_offset_z = LaunchConfiguration('cameraOffsetZ')
    check_terrain_connection = LaunchConfiguration('checkTerrainConn')
    use_rviz = LaunchConfiguration('use_rviz')
    max_speed = LaunchConfiguration('maxSpeed')
    autonomy_speed = LaunchConfiguration('autonomySpeed')
    goal_position_threshold = LaunchConfiguration('goalPosThre')
    goal_yaw_threshold = LaunchConfiguration('goalYawThre')

    declarations = [
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0'),
        DeclareLaunchArgument('checkTerrainConn', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('maxSpeed', default_value='1.0'),
        DeclareLaunchArgument('autonomySpeed', default_value='1.0'),
        DeclareLaunchArgument('goalPosThre', default_value='0.20'),
        DeclareLaunchArgument('goalYawThre', default_value='10.0'),
    ]

    local_planner = _include(
        'local_planner',
        'local_planner.launch',
        {
            'cameraOffsetZ': camera_offset_z,
            'maxSpeed': max_speed,
            'autonomySpeed': autonomy_speed,
            'goalPosThre': goal_position_threshold,
            'goalYawThre': goal_yaw_threshold,
        },
    )
    terrain_analysis = _include('terrain_analysis', 'terrain_analysis.launch')
    terrain_analysis_ext = _include(
        'terrain_analysis_ext',
        'terrain_analysis_ext.launch',
        {'checkTerrainConn': check_terrain_connection},
    )
    sensor_scan_generation = _include(
        'sensor_scan_generation', 'sensor_scan_generation.launch'
    )
    loam_interface = _include('loam_interface', 'loam_interface.launch')

    rviz_config = os.path.join(
        _get_vehicle_simulator_share(), 'rviz', 'vehicle_simulator.rviz'
    )
    delayed_rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                condition=IfCondition(use_rviz),
            )
        ],
    )

    return LaunchDescription(
        declarations
        + [
            local_planner,
            terrain_analysis,
            terrain_analysis_ext,
            sensor_scan_generation,
            loam_interface,
            delayed_rviz,
        ]
    )
