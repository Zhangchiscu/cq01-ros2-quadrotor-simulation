# 需求与实现追踪矩阵

状态字段沿用原始机器标记：`DONE`（已完成）、`PARTIAL`（部分完成）、`TODO`（待完成）、`BLOCKED`（受阻）。本表是开发阶段的追踪记录，不应被误认为最终验收状态。

## 必做项

| 编号 | 要求 | 状态 | 证据 |
|---|---|---|---|
| R1 | 自建 ROS2 工程 | DONE | Git 历史、独立包结构 |
| R2 | 四电机 RPM 输入的动力学节点 | DONE | drone_dynamics |
| R3 | odom、imu、tf、path 输出 | DONE | ROS2 话题检查 |
| R4 | 电机响应、推力力矩、平动转动、四元数积分 | DONE | 源码与悬停实验 |
| R5 | RPM 上下限保护 | DONE | RPM 饱和率 |
| R6 | 位置环、姿态环、电机混控器（mixer） | DONE | drone_controller |
| R7 | 推力、姿态、RPM 限幅和防炸机 | PARTIAL | 需做参数审计 |
| R8 | 地图与至少 5 个障碍物 | DONE | drone_map、drone_world |
| R9 | 安全避障与安全距离 | DONE | 标准避障、S 走廊 |
| R10 | 无人机、目标、轨迹、地图、路径可视化 | DONE | RViz |
| R11 | 连续 3–4 个目标 | DONE | 4/4 通过 |
| R12 | 误差、RPM、轨迹、最小净空曲线 | DONE | CSV/JSON/PNG |
| R13 | README 与一键运行 | PARTIAL | 最终整合待做 |
| R14 | 公开 Git 仓库 | TODO | GitHub |
| R15 | 6–10 页 PDF 报告 | TODO | report/ |
| R16 | 1–3 分钟录屏视频 | TODO | video/ |

## 全部明确加分项

| 编号 | 加分项 | 状态 |
|---|---|---|
| B1 | 参数文件统一配置 | PARTIAL |
| B2 | 风或外力扰动 | TODO |
| B3 | 传感器噪声和简化 IMU | PARTIAL |
| B4 | 点云、体素或局部感知 | PARTIAL |
| B5 | 多无人机 | TODO |
| B6 | 圆形轨迹、八字轨迹或航点列表 | PARTIAL |
| B7 | 单元测试或脚本化评测 | PARTIAL |
| B8 | Web/Qt/Python 地面站 | TODO |
| B9 | 与 pengyu_sim、MARSIM 比较 | TODO |

## 扩展主线

1. 多场景 3D 世界；
2. PointCloud2 与 三维体素；
3. 三维 A*／Theta*；
4. 路径平滑；
5. 局部感知、动态障碍与重规划；
6. 风、IMU、双无人机、轨迹输入；
7. PySide6 地面站；
8. CesiumJS 重庆渝中区视觉回放；
9. README、AI 说明、报告、视频、GitHub。

每个功能必须具备代码、参数、话题、启动文件、成功案例、边界案例和结果文件。
