"""ROS 2 adapter: plan once, export once, publish latched messages."""

from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

from .exceptions import PlanningError
from .exporters import (
    save_clean_map,
    save_compact_csv,
    save_detailed_csv,
    save_preview,
)
from .planner import plan_trajectory
from .ros_messages import (
    centerline_marker,
    occupancy_grid,
    path_message,
    speed_array,
    speed_marker,
    trajectory_data_array,
)
from .ros_parameters import declare_parameters, planner_config_from_node


class TrajectoryPlannerNode(Node):
    """One-shot global trajectory planner with transient-local publishers."""

    def __init__(self) -> None:
        super().__init__('trajectory_planner')
        declare_parameters(self)
        map_yaml = self._required_path('map_yaml')
        self.get_logger().info(f'Planning trajectory from {map_yaml}')
        self.trajectory = plan_trajectory(
            map_yaml, planner_config_from_node(self))
        csv_path, clean_yaml_path, clean_image_path = self._export_outputs(
            map_yaml)
        self._create_publishers()
        self._publish()
        self.get_logger().info(
            f'Published {len(self.trajectory.x)} points, '
            f'{self.trajectory.length:.2f} m, estimated lap '
            f'{self.trajectory.estimated_lap_time:.2f} s, direction '
            f'{self.trajectory.direction}; removed '
            f'{self.trajectory.map_data.removed_occupied_speckle_cells} '
            f'occupied speckle cells; clean map: {clean_yaml_path}, '
            f'{clean_image_path}; CSV: {csv_path}')

    def _required_path(self, parameter_name: str) -> Path:
        value = str(self.get_parameter(parameter_name).value)
        if not value:
            raise PlanningError(f'The {parameter_name} parameter is required')
        return Path(value).expanduser().resolve()

    def _export_outputs(self, map_yaml: Path) -> tuple[Path, Path, Path]:
        output_value = str(self.get_parameter('output_csv').value)
        output = (
            Path(output_value).expanduser()
            if output_value else map_yaml.with_name(
                f'{map_yaml.stem}_trajectory.csv'))
        exporter = (
            save_detailed_csv
            if bool(self.get_parameter('detailed_csv').value)
            else save_compact_csv)
        csv_path = exporter(self.trajectory, output)
        clean_yaml_value = str(self.get_parameter('clean_map_yaml').value)
        clean_yaml = (
            Path(clean_yaml_value).expanduser()
            if clean_yaml_value else map_yaml.with_name(
                f'{map_yaml.stem}_clean.yaml'))
        clean_image_value = str(self.get_parameter('clean_map_image').value)
        clean_image = (
            Path(clean_image_value).expanduser()
            if clean_image_value else clean_yaml.with_suffix('.pgm'))
        clean_yaml_path, clean_image_path = save_clean_map(
            self.trajectory.map_data, clean_yaml, clean_image)
        preview_value = str(self.get_parameter('preview_png').value)
        if preview_value:
            save_preview(self.trajectory, Path(preview_value).expanduser())
        return csv_path, clean_yaml_path, clean_image_path

    def _create_publishers(self) -> None:
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        publisher = self.create_publisher

        def parameter(name):
            return str(self.get_parameter(name).value)

        self.map_publisher = publisher(
            OccupancyGrid, parameter('map_topic'), qos)
        self.path_publisher = publisher(
            PathMessage, parameter('path_topic'), qos)
        self.speed_publisher = publisher(
            Float32MultiArray, parameter('speed_topic'), qos)
        self.data_publisher = publisher(
            Float32MultiArray, parameter('trajectory_data_topic'), qos)
        self.trajectory_marker_publisher = publisher(
            Marker, parameter('trajectory_marker_topic'), qos)
        self.centerline_marker_publisher = publisher(
            Marker, parameter('centerline_marker_topic'), qos)

    def _publish(self) -> None:
        from std_msgs.msg import Header

        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=str(self.get_parameter('frame_id').value),
        )
        self.map_publisher.publish(occupancy_grid(self.trajectory, header))
        self.path_publisher.publish(path_message(self.trajectory, header))
        self.speed_publisher.publish(speed_array(self.trajectory))
        self.data_publisher.publish(trajectory_data_array(self.trajectory))
        self.centerline_marker_publisher.publish(
            centerline_marker(self.trajectory, header))
        self.trajectory_marker_publisher.publish(
            speed_marker(self.trajectory, header))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (PlanningError, FileNotFoundError, OSError, ValueError) as error:
        if node:
            node.get_logger().fatal(str(error))
        else:
            print(f'trajectory_planner: {error}')
        raise
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
