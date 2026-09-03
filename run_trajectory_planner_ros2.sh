#!/usr/bin/env bash

set -e

# ======================== 所有参数都在这里 ========================

# 当前脚本所在目录：
# /home/plamephia/workspace/sim_ws/map_process
MAP_PROCESS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# ROS2 workspace：
# /home/plamephia/workspace/sim_ws
WORKSPACE_DIR="$(cd -- "${MAP_PROCESS_DIR}/.." && pwd)"

ROS_SETUP="/opt/ros/humble/setup.bash"

# 可选起点和方向；.nan 表示自动选择。
SEED_X=0.0
SEED_Y=0.0
SEED_YAW=.nan
REVERSE=false
DIRECTION=clockwise # 可选值：counterclockwise、clockwise

# 地图名称。地图文件应位于：
# ${MAP_PROCESS_DIR}/${MAP_NAME}/${MAP_NAME}.yaml
# 也可以把地图名称作为第一个参数传入，例如：
# ./run_trajectory_planner_ros2.sh Spielberg
MAP_NAME="${1:-map_202609032}"
MAP_DIR="${MAP_PROCESS_DIR}/${MAP_NAME}"
RUN_TIMESTAMP="$(date '+%Y%m%d_%H%M%S')"

# 输入、输出和显示。轨迹结果保存在对应地图目录中。
MAP_YAML="${MAP_DIR}/${MAP_NAME}.yaml"
CLEAN_MAP_YAML="${MAP_DIR}/${MAP_NAME}_clean.yaml"
CLEAN_MAP_IMAGE="${MAP_DIR}/${MAP_NAME}_clean.pgm"
OUTPUT_CSV="${MAP_DIR}/${MAP_NAME}_${DIRECTION}_${RUN_TIMESTAMP}.csv"
PREVIEW_PNG="${MAP_DIR}/${MAP_NAME}_${DIRECTION}_preview_${RUN_TIMESTAMP}.png"
PARAMS_YAML="${MAP_DIR}/${MAP_NAME}_${DIRECTION}_${RUN_TIMESTAMP}_params.yaml"
DETAILED_CSV=false
OPEN_RVIZ=true
USE_SIM_TIME=false

# ROS frame 和 topic。
FRAME_ID="map"
MAP_TOPIC="/map"
PATH_TOPIC="/planned_trajectory"
SPEED_TOPIC="/planned_speeds"
TRAJECTORY_DATA_TOPIC="/planned_trajectory_data"
TRAJECTORY_MARKER_TOPIC="/planned_trajectory_marker"
CENTERLINE_MARKER_TOPIC="/planned_centerline_marker"

# 车辆尺寸和轨迹采样，单位 m。
SPACING=0.20
CENTERLINE_SMOOTHING=0.25
VEHICLE_WIDTH=0.30
WALL_MARGIN=0.05
# 仅过滤被自由空间完全包围的占用噪点，单位为像素；0 表示禁用。
# map_1788291473 的赛道内残留由 1 至 2 像素的连通块组成。
MAX_OCCUPIED_SPECKLE_AREA=20

# 速度和加速度限制，单位 m/s、m/s^2。
MAX_SPEED=6.0
MIN_SPEED=0.5
MAX_LATERAL_ACCEL=5.0
MAX_ACCEL=3.0
MAX_DECEL=3.0
LATERAL_ACCEL_SAFETY_FACTOR=0.90
FRONT_GRIP_FACTOR=1.00
REAR_GRIP_FACTOR=0.95
DRIVE_FRONT_FRACTION=0.50
BRAKE_FRONT_FRACTION=0.60

# Racing line 优化。
CURVATURE_WEIGHT=1.0
CURVATURE_SMOOTH_WEIGHT=0.35
LENGTH_WEIGHT=0.04
OFFSET_SMOOTH_WEIGHT=0.15
CENTER_WEIGHT=0.001
CORRIDOR_FRACTION=0.92
MAX_OPTIMIZATION_ITERATIONS=100

# 速度曲线和最短时间优化。
MAX_VELOCITY_ITERATIONS=100
TIME_OPTIMIZATION_MODES=8
MAX_TIME_OPTIMIZATION_ITERATIONS=25
TIME_OPTIMIZATION_PASSES=2
TIME_OPTIMIZATION_STEP=0.12
TIME_OFFSET_REGULARIZATION=0.015

# ======================== 参数到这里结束 ==========================

if [[ ! -f "$MAP_YAML" ]]; then
    echo "错误：找不到地图配置文件：$MAP_YAML" >&2
    exit 1
fi

source "$ROS_SETUP"

# 必须在 ROS2 workspace 根目录进行 colcon build
cd "$WORKSPACE_DIR"

colcon build \
    --symlink-install \
    --packages-select trajectory_planner

source "${WORKSPACE_DIR}/install/setup.bash"

# 保存本次运行所使用的参数。文件名与 CSV 共用同一个时间戳，方便一一对应。
# YAML 字符串使用单引号，并把字符串中的单引号转义为两个单引号。
yaml_quote() {
    local value="${1//\'/\'\'}"
    printf "'%s'" "$value"
}

{
    printf "run:\n"
    printf "  timestamp: %s\n" "$(yaml_quote "$RUN_TIMESTAMP")"
    printf "  map_name: %s\n" "$(yaml_quote "$MAP_NAME")"
    printf "  params_yaml: %s\n" "$(yaml_quote "$PARAMS_YAML")"
    printf "\nparameters:\n"
    printf "  map_yaml: %s\n" "$(yaml_quote "$MAP_YAML")"
    printf "  clean_map_yaml: %s\n" "$(yaml_quote "$CLEAN_MAP_YAML")"
    printf "  clean_map_image: %s\n" "$(yaml_quote "$CLEAN_MAP_IMAGE")"
    printf "  output_csv: %s\n" "$(yaml_quote "$OUTPUT_CSV")"
    printf "  preview_png: %s\n" "$(yaml_quote "$PREVIEW_PNG")"
    printf "  detailed_csv: %s\n" "$DETAILED_CSV"
    printf "  open_rviz: %s\n" "$OPEN_RVIZ"
    printf "  use_sim_time: %s\n" "$USE_SIM_TIME"
    printf "  frame_id: %s\n" "$(yaml_quote "$FRAME_ID")"
    printf "  map_topic: %s\n" "$(yaml_quote "$MAP_TOPIC")"
    printf "  path_topic: %s\n" "$(yaml_quote "$PATH_TOPIC")"
    printf "  speed_topic: %s\n" "$(yaml_quote "$SPEED_TOPIC")"
    printf "  trajectory_data_topic: %s\n" "$(yaml_quote "$TRAJECTORY_DATA_TOPIC")"
    printf "  trajectory_marker_topic: %s\n" "$(yaml_quote "$TRAJECTORY_MARKER_TOPIC")"
    printf "  centerline_marker_topic: %s\n" "$(yaml_quote "$CENTERLINE_MARKER_TOPIC")"
    printf "  spacing: %s\n" "$SPACING"
    printf "  centerline_smoothing: %s\n" "$CENTERLINE_SMOOTHING"
    printf "  vehicle_width: %s\n" "$VEHICLE_WIDTH"
    printf "  wall_margin: %s\n" "$WALL_MARGIN"
    printf "  max_occupied_speckle_area: %s\n" "$MAX_OCCUPIED_SPECKLE_AREA"
    printf "  max_speed: %s\n" "$MAX_SPEED"
    printf "  min_speed: %s\n" "$MIN_SPEED"
    printf "  max_lateral_accel: %s\n" "$MAX_LATERAL_ACCEL"
    printf "  max_accel: %s\n" "$MAX_ACCEL"
    printf "  max_decel: %s\n" "$MAX_DECEL"
    printf "  lateral_accel_safety_factor: %s\n" "$LATERAL_ACCEL_SAFETY_FACTOR"
    printf "  front_grip_factor: %s\n" "$FRONT_GRIP_FACTOR"
    printf "  rear_grip_factor: %s\n" "$REAR_GRIP_FACTOR"
    printf "  drive_front_fraction: %s\n" "$DRIVE_FRONT_FRACTION"
    printf "  brake_front_fraction: %s\n" "$BRAKE_FRONT_FRACTION"
    printf "  curvature_weight: %s\n" "$CURVATURE_WEIGHT"
    printf "  curvature_smooth_weight: %s\n" "$CURVATURE_SMOOTH_WEIGHT"
    printf "  length_weight: %s\n" "$LENGTH_WEIGHT"
    printf "  offset_smooth_weight: %s\n" "$OFFSET_SMOOTH_WEIGHT"
    printf "  center_weight: %s\n" "$CENTER_WEIGHT"
    printf "  corridor_fraction: %s\n" "$CORRIDOR_FRACTION"
    printf "  max_optimization_iterations: %s\n" "$MAX_OPTIMIZATION_ITERATIONS"
    printf "  max_velocity_iterations: %s\n" "$MAX_VELOCITY_ITERATIONS"
    printf "  time_optimization_modes: %s\n" "$TIME_OPTIMIZATION_MODES"
    printf "  max_time_optimization_iterations: %s\n" "$MAX_TIME_OPTIMIZATION_ITERATIONS"
    printf "  time_optimization_passes: %s\n" "$TIME_OPTIMIZATION_PASSES"
    printf "  time_optimization_step: %s\n" "$TIME_OPTIMIZATION_STEP"
    printf "  time_offset_regularization: %s\n" "$TIME_OFFSET_REGULARIZATION"
    printf "  seed_x: %s\n" "$SEED_X"
    printf "  seed_y: %s\n" "$SEED_Y"
    printf "  seed_yaw: %s\n" "$SEED_YAW"
    printf "  reverse: %s\n" "$REVERSE"
    printf "  direction: %s\n" "$(yaml_quote "$DIRECTION")"
} > "$PARAMS_YAML"

echo "本次运行参数已保存：$PARAMS_YAML"

exec ros2 launch trajectory_planner trajectory_planner.launch.py \
    map_yaml:="$MAP_YAML" \
    clean_map_yaml:="$CLEAN_MAP_YAML" \
    clean_map_image:="$CLEAN_MAP_IMAGE" \
    output_csv:="$OUTPUT_CSV" \
    preview_png:="$PREVIEW_PNG" \
    detailed_csv:="$DETAILED_CSV" \
    open_rviz:="$OPEN_RVIZ" \
    use_sim_time:="$USE_SIM_TIME" \
    frame_id:="$FRAME_ID" \
    map_topic:="$MAP_TOPIC" \
    path_topic:="$PATH_TOPIC" \
    speed_topic:="$SPEED_TOPIC" \
    trajectory_data_topic:="$TRAJECTORY_DATA_TOPIC" \
    trajectory_marker_topic:="$TRAJECTORY_MARKER_TOPIC" \
    centerline_marker_topic:="$CENTERLINE_MARKER_TOPIC" \
    spacing:="$SPACING" \
    centerline_smoothing:="$CENTERLINE_SMOOTHING" \
    vehicle_width:="$VEHICLE_WIDTH" \
    wall_margin:="$WALL_MARGIN" \
    max_occupied_speckle_area:="$MAX_OCCUPIED_SPECKLE_AREA" \
    max_speed:="$MAX_SPEED" \
    min_speed:="$MIN_SPEED" \
    max_lateral_accel:="$MAX_LATERAL_ACCEL" \
    max_accel:="$MAX_ACCEL" \
    max_decel:="$MAX_DECEL" \
    lateral_accel_safety_factor:="$LATERAL_ACCEL_SAFETY_FACTOR" \
    front_grip_factor:="$FRONT_GRIP_FACTOR" \
    rear_grip_factor:="$REAR_GRIP_FACTOR" \
    drive_front_fraction:="$DRIVE_FRONT_FRACTION" \
    brake_front_fraction:="$BRAKE_FRONT_FRACTION" \
    curvature_weight:="$CURVATURE_WEIGHT" \
    curvature_smooth_weight:="$CURVATURE_SMOOTH_WEIGHT" \
    length_weight:="$LENGTH_WEIGHT" \
    offset_smooth_weight:="$OFFSET_SMOOTH_WEIGHT" \
    center_weight:="$CENTER_WEIGHT" \
    corridor_fraction:="$CORRIDOR_FRACTION" \
    max_optimization_iterations:="$MAX_OPTIMIZATION_ITERATIONS" \
    max_velocity_iterations:="$MAX_VELOCITY_ITERATIONS" \
    time_optimization_modes:="$TIME_OPTIMIZATION_MODES" \
    max_time_optimization_iterations:="$MAX_TIME_OPTIMIZATION_ITERATIONS" \
    time_optimization_passes:="$TIME_OPTIMIZATION_PASSES" \
    time_optimization_step:="$TIME_OPTIMIZATION_STEP" \
    time_offset_regularization:="$TIME_OFFSET_REGULARIZATION" \
    seed_x:="$SEED_X" \
    seed_y:="$SEED_Y" \
    seed_yaw:="$SEED_YAW" \
    reverse:="$REVERSE" \
    direction:="$DIRECTION"
