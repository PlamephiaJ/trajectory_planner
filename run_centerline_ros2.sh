#!/usr/bin/env bash

set -euo pipefail

# 只从栅格地图提取赛道中心线，不运行 racing-line 或最短时间线路优化。
# 用法：./run_centerline_ros2.sh [地图名称]

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
MAPS_DIR="${SCRIPT_DIR}/maps"
ROS_SETUP="/opt/ros/humble/setup.bash"

MAP_NAME="${1:-a219_0922}"
MAP_DIR="${MAPS_DIR}/${MAP_NAME}"
MAP_YAML="${MAP_DIR}/${MAP_NAME}.yaml"
RUN_TIMESTAMP="$(date '+%Y%m%d_%H%M%S')"
RUN_DIR="${MAP_DIR}/${RUN_TIMESTAMP}_centerline"

OUTPUT_CSV="${RUN_DIR}/${MAP_NAME}_centerline.csv"
PREVIEW_PNG="${RUN_DIR}/${MAP_NAME}_centerline_preview.png"
CLEAN_MAP_YAML="${RUN_DIR}/${MAP_NAME}_clean.yaml"
CLEAN_MAP_IMAGE="${RUN_DIR}/${MAP_NAME}_clean.pgm"

# 中心线提取参数。
SPACING=0.20
CENTERLINE_SMOOTHING=0.25
VEHICLE_WIDTH=0.30
WALL_MARGIN=0.05
MAX_OCCUPIED_SPECKLE_AREA=20
SEED_X=0.0
SEED_Y=0.0
DIRECTION=clockwise

OPEN_RVIZ=false
USE_SIM_TIME=false

if [[ ! -f "$MAP_YAML" ]]; then
    echo "错误：找不到地图配置文件：$MAP_YAML" >&2
    exit 1
fi

# ROS 的 setup.bash 会读取一些可能尚未定义的环境变量，因此 source 时
# 临时关闭 nounset，随后恢复。
set +u
source "$ROS_SETUP"
set -u
cd "$WORKSPACE_DIR"
colcon build --symlink-install --packages-select trajectory_planner
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

mkdir -p "$RUN_DIR"

exec ros2 launch trajectory_planner trajectory_planner.launch.py \
    map_yaml:="$MAP_YAML" \
    clean_map_yaml:="$CLEAN_MAP_YAML" \
    clean_map_image:="$CLEAN_MAP_IMAGE" \
    output_csv:="$OUTPUT_CSV" \
    preview_png:="$PREVIEW_PNG" \
    centerline_only:=true \
    spacing:="$SPACING" \
    centerline_smoothing:="$CENTERLINE_SMOOTHING" \
    vehicle_width:="$VEHICLE_WIDTH" \
    wall_margin:="$WALL_MARGIN" \
    max_occupied_speckle_area:="$MAX_OCCUPIED_SPECKLE_AREA" \
    seed_x:="$SEED_X" \
    seed_y:="$SEED_Y" \
    direction:="$DIRECTION" \
    min_turning_radius:=0.0 \
    open_rviz:="$OPEN_RVIZ" \
    use_sim_time:="$USE_SIM_TIME"
