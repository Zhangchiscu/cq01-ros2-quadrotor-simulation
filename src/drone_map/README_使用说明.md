# drone_map

提供：

- 6 个三维静态障碍物；
- 0.25 m 安全膨胀区；
- `/drone/obstacle_markers`；
- `/drone/obstacle_grid`；
- 线段—膨胀障碍物碰撞检测基础函数。

## 安装

```bash
cp /mnt/hgfs/VMShare/drone_map_bundle.zip ~/Downloads/
cd ~/Downloads
unzip -o drone_map_bundle.zip
cd drone_map_bundle
chmod +x install.sh
./install.sh
```

## 测试

```bash
source ~/drone_task/your_drone_sim_ws/install/setup.bash
ros2 launch drone_map static_map.launch.py
```

正常日志包含：

```text
Static obstacle map started
Obstacles: 6, safety margin: 0.25 m
Self-test: direct path (0,0)->(6,0) is blocked by center_block
```
