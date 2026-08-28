# trajectory_planner

标准 ROS 2 `ament_python` 包：从 ROS PGM/YAML 地图生成闭环赛道轨迹、速度曲线、
CSV、预览图以及可直接在 RViz2 查看的一组 transient-local topic。

## 目录

```text
trajectory_planner/
├── config/                 # ROS 与算法参数
├── launch/                 # planner + RViz2
├── rviz/                   # 已配置的 RViz2 布局
├── test/                   # ROS 无关的算法测试
└── trajectory_planner/
    ├── config.py           # PlannerConfig
    ├── map_loader.py       # PGM/YAML
    ├── centerline.py       # 闭环与中心线
    ├── geometry.py         # 曲率、宽度、安全距离
    ├── optimization.py     # racing line
    ├── velocity.py         # 速度曲线
    ├── planner.py          # 顶层编排
    ├── exporters.py        # CSV 和 PNG
    ├── ros_messages.py     # ROS 消息转换
    └── node.py             # ROS 节点生命周期
```

## ROS 2

通常只需编辑工作区根目录的 `run_trajectory_planner_ros2.sh`：全部车辆、速度、
优化、输出和 topic 参数都集中在文件顶部，文件底部是一条完整的 `ros2 launch`
命令。随后直接运行：

```bash
./run_trajectory_planner_ros2.sh
```

下面是等价的手动启动方式：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select trajectory_planner
source install/setup.bash
ros2 launch trajectory_planner trajectory_planner.launch.py \
  map_yaml:=$PWD/map_20260827.yaml \
  output_csv:=$PWD/map_20260827_trajectory.csv \
  preview_png:=$PWD/map_20260827_trajectory_preview.png
```

RViz2 默认自动打开，显示 `/map`、蓝色中心线和按红（慢）到绿（快）着色的最终
轨迹。无图形界面时增加 `open_rviz:=false`。

## 离线 Python

```bash
python3 -m trajectory_planner map.yaml \
  --map-image map.pgm --output trajectory.csv --preview preview.png
```

默认 CSV 只有 `x,y,speed`；增加 `--detailed-csv` 可输出全部诊断字段。
