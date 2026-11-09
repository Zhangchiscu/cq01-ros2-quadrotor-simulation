# CQ01 需求补全扩展模块

这是在核心 CQ01 系统冻结之后增加的独立 ROS 2 软件包。

## 1. 传感器噪声仿真

输入：
- `/drone/odom`（`nav_msgs/msg/Odometry`）
- `/drone/imu`（`sensor_msgs/msg/Imu`）

输出：
- `/drone/odom_noisy`
- `/drone/imu_noisy`
- `/cq01_bonus/sensor_noise/status`

噪声模型：
- 零均值高斯位置噪声；
- 零均值高斯线速度／角速度噪声；
- 欧拉角扰动后进行四元数归一化；
- IMU 角速度与线加速度噪声；
- 固定随机种子，以保证实验可复现。

默认参数位于 `config/requirement_closure.yaml`。

冻结后的主控制器**没有修改**。本模块属于可选的传感器模型实验，用于提供“传感器噪声、IMU 简化模型”加分项的验证证据。

## 2. 局部感知范围仿真

输入：
- `/drone/odom`
- `/map/obstacles`（`visualization_msgs/msg/MarkerArray`）

输出：
- `/map/local_obstacles`
- `/cq01_bonus/perception_range`
- `/cq01_bonus/local_perception/status`

该节点仅保留障碍物表面近似距离落在配置半径以内的障碍物。默认半径为 12 m。同时发布由三个圆环构成的线框球体，便于在 RViz 中展示验证结果。

本模块在不改变冻结版全局规划器的前提下，满足可选的“局部感知范围模拟”方向。

## 运行方式

首先启动正常的 CQ01 任务，再打开另一个终端：

```bash
source ~/drone_task/your_drone_sim_ws/install/setup.bash
ros2 launch cq01_requirement_closure requirement_closure.launch.py
```

然后运行随安装包提供的审计脚本。
