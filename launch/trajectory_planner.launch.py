"""Launch the planner and an optional preconfigured RViz2 instance."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PARAMETER_DEFAULTS = {
    'clean_map_yaml': '',
    'clean_map_image': '',
    'output_csv': '',
    'preview_png': '',
    'detailed_csv': 'false',
    'frame_id': 'map',
    'map_topic': '/map',
    'path_topic': '/planned_trajectory',
    'speed_topic': '/planned_speeds',
    'trajectory_data_topic': '/planned_trajectory_data',
    'trajectory_marker_topic': '/planned_trajectory_marker',
    'centerline_marker_topic': '/planned_centerline_marker',
    'spacing': '0.20',
    'centerline_smoothing': '0.25',
    'vehicle_width': '0.30',
    'wall_margin': '0.05',
    'max_occupied_speckle_area': '2',
    'max_speed': '6.0',
    'min_speed': '0.5',
    'max_lateral_accel': '7.0',
    'max_accel': '3.0',
    'max_decel': '6.0',
    'lateral_accel_safety_factor': '0.90',
    'front_grip_factor': '1.00',
    'rear_grip_factor': '0.95',
    'drive_front_fraction': '0.00',
    'brake_front_fraction': '0.60',
    'curvature_weight': '1.0',
    'curvature_smooth_weight': '0.35',
    'length_weight': '0.04',
    'offset_smooth_weight': '0.15',
    'center_weight': '0.001',
    'corridor_fraction': '0.92',
    'max_optimization_iterations': '100',
    'max_velocity_iterations': '100',
    'time_optimization_modes': '8',
    'max_time_optimization_iterations': '25',
    'time_optimization_passes': '2',
    'time_optimization_step': '0.12',
    'time_offset_regularization': '0.015',
    'seed_x': '.nan',
    'seed_y': '.nan',
    'seed_yaw': '.nan',
    'direction': 'auto',
    'reverse': 'false',
    'use_sim_time': 'false',
}


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('trajectory_planner'))
    open_rviz = LaunchConfiguration('open_rviz')
    parameters = {
        name: LaunchConfiguration(name)
        for name in ('map_yaml', *PARAMETER_DEFAULTS)
    }
    declarations = [
        DeclareLaunchArgument('map_yaml', description='Absolute map YAML path'),
        DeclareLaunchArgument('open_rviz', default_value='true'),
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in PARAMETER_DEFAULTS.items()
        ],
    ]
    return LaunchDescription([
        *declarations,
        Node(
            package='trajectory_planner',
            executable='trajectory_planner_node',
            name='trajectory_planner',
            output='screen',
            parameters=[
                str(share / 'config' / 'trajectory_planner.yaml'),
                parameters,
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='trajectory_planner_rviz',
            output='screen',
            arguments=['-d', str(share / 'rviz' / 'trajectory_planner.rviz')],
            parameters=[{'use_sim_time': parameters['use_sim_time']}],
            condition=IfCondition(open_rviz),
        ),
    ])
