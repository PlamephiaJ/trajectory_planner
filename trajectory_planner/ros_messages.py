"""Pure conversion of Trajectory data into ROS messages."""

import math

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import ColorRGBA, Float32MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker

from .models import Trajectory


def occupancy_grid(trajectory: Trajectory, header) -> OccupancyGrid:
    map_data = trajectory.map_data
    message = OccupancyGrid()
    message.header = header
    message.info.map_load_time = header.stamp
    message.info.resolution = float(map_data.resolution)
    message.info.width = int(map_data.image.shape[1])
    message.info.height = int(map_data.image.shape[0])
    message.info.origin.position.x = float(map_data.origin_x)
    message.info.origin.position.y = float(map_data.origin_y)
    message.info.origin.orientation.z = math.sin(0.5 * map_data.origin_yaw)
    message.info.origin.orientation.w = math.cos(0.5 * map_data.origin_yaw)
    occupancy = np.full(map_data.image.shape, 100, dtype=np.int8)
    occupancy[map_data.free] = 0
    occupancy[np.abs(map_data.image.astype(np.int16) - 205) <= 1] = -1
    message.data = np.flipud(occupancy).reshape(-1).astype(int).tolist()
    return message


def path_message(trajectory: Trajectory, header) -> Path:
    message = Path()
    message.header = header
    for x, y, yaw in zip(trajectory.x, trajectory.y, trajectory.yaw):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(0.5 * float(yaw))
        pose.pose.orientation.w = math.cos(0.5 * float(yaw))
        message.poses.append(pose)
    return message


def speed_array(trajectory: Trajectory) -> Float32MultiArray:
    message = Float32MultiArray()
    message.layout.dim.append(MultiArrayDimension(
        label='trajectory_points',
        size=len(trajectory.speed),
        stride=len(trajectory.speed),
    ))
    message.data = [float(speed) for speed in trajectory.speed]
    return message


def trajectory_data_array(trajectory: Trajectory) -> Float32MultiArray:
    message = Float32MultiArray()
    point_count = len(trajectory.x)
    message.layout.dim.extend((
        MultiArrayDimension(
            label='trajectory_points', size=point_count,
            stride=point_count * 9),
        MultiArrayDimension(
            label='s_x_y_yaw_curvature_speed_left_right_offset',
            size=9, stride=9),
    ))
    distance = 0.0
    for index in range(point_count):
        message.data.extend((
            distance,
            float(trajectory.x[index]),
            float(trajectory.y[index]),
            float(trajectory.yaw[index]),
            float(trajectory.curvature[index]),
            float(trajectory.speed[index]),
            float(trajectory.left_width[index]),
            float(trajectory.right_width[index]),
            float(trajectory.lateral_offset[index]),
        ))
        distance += float(trajectory.segment_length[index])
    return message


def centerline_marker(trajectory: Trajectory, header) -> Marker:
    marker = _line_marker(header, 'track_centerline', 0, 0.035)
    marker.color = ColorRGBA(r=0.05, g=0.65, b=1.0, a=0.85)
    marker.points = [
        Point(x=float(x), y=float(y), z=0.03)
        for x, y in zip(trajectory.center_x, trajectory.center_y)
    ]
    marker.points.append(marker.points[0])
    return marker


def speed_marker(trajectory: Trajectory, header) -> Marker:
    marker = _line_marker(header, 'optimal_trajectory', 0, 0.075)
    minimum = float(np.min(trajectory.speed))
    span = max(float(np.max(trajectory.speed)) - minimum, 1.0e-6)
    for x, y, speed in zip(trajectory.x, trajectory.y, trajectory.speed):
        marker.points.append(Point(x=float(x), y=float(y), z=0.05))
        ratio = (float(speed) - minimum) / span
        marker.colors.append(ColorRGBA(
            r=float(1.0 - ratio), g=float(ratio), b=0.05, a=1.0))
    marker.points.append(marker.points[0])
    marker.colors.append(marker.colors[0])
    return marker


def _line_marker(header, namespace: str, marker_id: int, width: float) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    return marker
