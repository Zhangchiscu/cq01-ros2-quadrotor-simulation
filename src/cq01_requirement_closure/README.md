# CQ01 Requirement Closure Add-ons

Independent ROS 2 package added after the core CQ01 system was frozen.

## 1. Sensor-noise simulation

Inputs:
- `/drone/odom` (`nav_msgs/msg/Odometry`)
- `/drone/imu` (`sensor_msgs/msg/Imu`)

Outputs:
- `/drone/odom_noisy`
- `/drone/imu_noisy`
- `/cq01_bonus/sensor_noise/status`

Noise model:
- zero-mean Gaussian position noise;
- zero-mean Gaussian linear/angular velocity noise;
- Euler-angle perturbation followed by quaternion normalization;
- IMU angular-rate and linear-acceleration noise;
- fixed random seed for reproducible experiments.

Default values are in `config/requirement_closure.yaml`.

The frozen main controller is **not** modified. This module is an optional
sensor-model experiment and evidence for the "传感器噪声、IMU 简化模型" item.

## 2. Local perception-range simulation

Inputs:
- `/drone/odom`
- `/map/obstacles` (`visualization_msgs/msg/MarkerArray`)

Outputs:
- `/map/local_obstacles`
- `/cq01_bonus/perception_range`
- `/cq01_bonus/local_perception/status`

The node keeps only obstacles whose approximate surface distance to the
vehicle is within the configured radius. The default radius is 12 m.
A three-ring wireframe sphere is published for RViz evidence.

This satisfies the optional "局部感知范围模拟" direction without changing
the frozen global planner.

## Run

First start the normal CQ01 mission. Then in another terminal:

```bash
source ~/drone_task/your_drone_sim_ws/install/setup.bash
ros2 launch cq01_requirement_closure requirement_closure.launch.py
```

Then run the audit script supplied with the installer bundle.
