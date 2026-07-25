# AI 使用说明（ai_usage.md）

## 1. 使用的 AI 工具

本项目开发过程中主要使用了以下 AI 工具：

1. **ChatGPT**：用于课程要求拆解、ROS 2 工程结构设计、代码脚手架生成、故障诊断、实验结果分析、测试脚本与提交材料整理。
2. **Ollama + Qwen3 4B（CPU 本地运行）**：曾尝试作为实验结果的语义评价器。由于本地虚拟机未暴露 NVIDIA GPU，推理延迟较高，且该功能对课程核心闭环贡献有限，因此后续停止使用。
3. **RAPTOR 论文阅读辅助**：使用 AI 协助理解论文中的 recurrent policy、teacher/student distillation、domain randomization、on-policy distribution shift 等概念。相关神经网络控制实验仅作为拓展探索，未并入最终课程运行链。

AI 的作用定位为：**辅助设计、编码、调试与复盘，不替代本人对动力学公式、控制接口、ROS 2 topic、实验结果和最终验收的确认。**

---

## 2. 关键 AI 交互摘要

下面列出本项目中具有代表性的 AI 辅助过程。实际交互次数远多于 8 次，此处保留对最终工程最有影响的部分。

### 交互 1：课程要求拆解与任务矩阵

**目标：** 将课程 PDF 中的动力学、控制、规划、可视化、验收场景、加分项和提交要求拆成可执行清单。

**AI 辅助内容：**
- 将要求划分为动力学、控制器、3D 规划、地图、可视化、实验、加分项和提交材料。
- 建议建立“硬性要求 / 加分项 / 证据材料”三层矩阵。
- 提醒不能只完成复杂重庆场景，还需要保留老师指定的悬停、单目标、多目标等基础验收证据。

**本人确认：**
- 按课程原始要求逐条核对。
- 最终保留 teacher-exact acceptance：悬停 `(0,0,1.5)`、单目标 `(2,1,1.5)`、四目标连续飞行等独立证据。

---

### 交互 2：ROS 2 工程模块化与一键启动

**目标：** 将已有 `drone_dynamics`、`drone_controller`、`drone_planner_3d`、地图和任务节点组织成可重复启动的工程。

**AI 辅助内容：**
- 建议按 dynamics / controller / planner / mission / visualization / evaluation 分层。
- 生成和修改 launch、shell 脚本、结果目录和自动化检查逻辑。
- 多次根据实际终端日志修复包构建、参数文件和 launch 时序问题。

**本人确认：**
- 每次修改均在 Ubuntu 22.04 + ROS 2 Humble 虚拟机中实际编译和运行。
- 以 `colcon build`、节点启动日志、topic 输出和最终闭环运行结果作为是否接受修改的依据。

---

### 交互 3：重庆山城场景与复杂任务设计

**目标：** 在课程功能基础上增加具有展示效果的重庆山城综合场景。

**AI 辅助内容：**
- 协助规划山地起伏、高层建筑、桥下/峡谷、低空与高空任务的组织方式。
- 对“只爬坡、不体现课程要求”的早期方案提出修改建议。
- 建议将课程要求中的避障、扰动、动态重规划、轨迹跟踪统一嵌入重庆背景，而不是只追求建筑外观还原。

**本人确认：**
- 根据 RViz 实际显示效果反复判断地形是否足够明显、建筑是否影响避障展示。
- 最终以 P0-P15 综合任务和独立验收结果验证场景不是纯视觉背景。

---

### 交互 4：3D A* 与动态重规划调试

**目标：** 完成静态障碍避障、狭窄通道、动态障碍和重规划。

**AI 辅助内容：**
- 协助分析 26 邻域 3D A*、路径简化、waypoint 输出和动态 occupancy 更新逻辑。
- 建议使用日志记录规划时延、路径分支距离、动态障碍触发次数。
- 根据多次实验结果分析重规划延迟与成功/失败原因。

**本人确认：**
- 通过实际运行日志确认 A* 路径产生。
- 通过独立 `course_clearance` 和 integrated evaluator 检查静态障碍安全距离、桥下/峡谷通过和动态重规划。
- 不以“RViz 看起来绕开了”作为唯一判断依据。

---

### 交互 5：风扰动、外力扰动与恢复评价

**目标：** 满足扰动加分项，并让扰动效果能够被老师观察和量化。

**AI 辅助内容：**
- 协助设计低空强风、外力扰动、爬升扰动和高空风扰动阶段。
- 建议增加扰动前后偏差、恢复过程、replan latency 和电机 RPM 响应记录。
- 在“扰动可视化不明显”时，建议用状态、轨迹和曲线证据，而不是只依赖肉眼观察。

**本人确认：**
- 实际运行并检查扰动阶段是否改变无人机轨迹和控制输出。
- 保存高空风、爬升扰动等独立 `summary.json` 和响应曲线。

---

### 交互 6：教师兼容 ROS 2 接口

**目标：** 保证最终系统具有课程要求的标准 topic / TF 接口。

**AI 辅助内容：**
- 协助设计 teacher interface bridge，仅提供兼容别名，不介入控制闭环。
- 建议审计 `/drone/motor_rpm_cmd`、`/drone/odom`、`/drone/imu`、`/drone/path`、`/drone/goal`、`/map/obstacles`、`/drone/reference`、`/drone/waypoints`、`/drone/safe_goal` 以及 `map -> base_link` TF。
- 特别检查 bridge 不应成为 `/drone/motor_rpm_cmd` 的控制发布者。

**本人确认：**
- 使用 ROS 2 topic graph、publisher/subscriber 信息和 TF 实际检查。
- 教师接口只作为兼容层，不替代 controller/planner。

---

### 交互 7：传感器闭环、噪声和局部感知

**目标：** 增强传感器模型，并补齐“传感器噪声 / IMU 简化模型 / 局部感知范围模拟”。

**AI 辅助内容：**
- 协助建立 `/drone/odom_noisy`、`/drone/imu_noisy` 的高斯噪声模型。
- 协助实现 12 m 局部 PointCloud2 裁剪与感知范围 Marker。
- 在测试过程中根据日志逐步定位：
  1. MarkerArray 与 PointCloud2 同 topic 不兼容；
  2. YAML 参数文件格式错误；
  3. `/map/obstacles` 为 `RELIABLE + TRANSIENT_LOCAL`，旧订阅为 `BEST_EFFORT + VOLATILE` 导致静态地图收不到。
- 最终将订阅 QoS 修正为 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`。

**本人确认：**
- 实际 audit 中确认噪声样本数持续增加。
- 确认 `/map/local_obstacles_cloud` 类型为 `sensor_msgs/msg/PointCloud2`。
- 实际局部点云 width 为非零值，证明不是只创建了空 topic。

---

### 交互 8：参数 profile 与鲁棒性比较

**目标：** 满足参数文件和不同机体参数的加分项。

**AI 辅助内容：**
- 协助建立 nominal / heavy 参数 profile。
- 建议 heavy 保持场景和控制逻辑不变，只修改质量与惯量，以形成可解释的对比实验。
- 生成参数比较和结果汇总脚本。

**本人确认：**
- 验证 nominal 与 heavy 参数文件实际加载。
- 对比 hover RPM、轨迹和闭环结果是否符合物理趋势。

---

### 交互 9：稳定性指标、自动记录与图表

**目标：** 满足老师要求的定量结果和稳定性展示。

**AI 辅助内容：**
- 协助统一 recorder 和自动出图。
- 整理位置误差、四电机 RPM、实际轨迹、规划轨迹、最小障碍物距离、飞行高度等结果。
- 建议最终使用独立 summary / metrics，而不是从终端截图人工估计数值。

**本人确认：**
- 检查生成的 PNG、CSV、JSON 是否与实际运行一致。
- Final Acceptance 中要求的六项最低验收和稳定性证据均从真实结果目录归档。

---

### 交互 10：RAPTOR-inspired 端到端电机控制探索

**目标：** 探索是否可以进一步实现“状态/目标 -> 四电机 RPM”的神经控制策略。

**AI 辅助内容：**
- 协助尝试 behavioral cloning、delta-RPM、100 Hz temporal GRU、on-policy corrective distillation、Safe-DAgger、persistent hidden state、局部 torque residual 等方案。
- 协助设计离线 MAE、autoregressive replay、Jacobian、启动时序和纯 AI hover 测试。

**本人确认与否决：**
- 多个模型即使离线 MAE 较低，纯 AI 闭环仍会在短时间内超过最大倾角。
- 进一步诊断发现闭环稳定性、分布偏移、时序状态和交叉耦合问题不能仅靠继续堆训练解决。
- 最终本人决定**及时止损**：该实验仅作为研究探索保留，**不合并到课程最终运行链，也不声称复现 RAPTOR**。

这一失败过程也是 AI 使用中最重要的反例：AI 可以持续提出方案，但最终是否可靠必须由实际闭环实验决定。

---

### 交互 11：最终验收证据聚合

**目标：** 防止复杂主任务掩盖课程最低验收项。

**AI 辅助内容：**
- 根据已有结果 inventory，自动寻找已通过的：
  - teacher-exact hover；
  - teacher-exact single goal；
  - 四 waypoint multi-goal；
  - P0-P15 static obstacle clearance；
  - bridge / narrow corridor；
  - required stability figures；
  - quantitative metrics；
  - sensor-noise / local perception audit。
- 生成统一 `final_acceptance` 目录和 `acceptance_summary.json`。

**本人确认：**
- 最终执行结果：
  - `FINAL ACCEPTANCE RESULT: pass=True`
  - `failed=[]`
- 多无人机按照最终项目决策排除。
- E2E motor-AI 实验明确排除在最终运行证据之外。

---

## 3. AI 参与过的主要模块

AI 对以下内容提供过不同程度的代码生成、修改建议或调试辅助：

- ROS 2 launch / shell 启动脚本；
- 3D A* 周边任务组织与自动评价逻辑；
- 重庆综合场景任务编排；
- 动态障碍和重规划测试；
- 风扰动 / 外力扰动评价；
- recorder、CSV、JSON、自动出图；
- 参数 profile 与批量测试；
- teacher interface bridge / audit；
- 传感器噪声和局部 PointCloud2 感知模块；
- Final Acceptance 证据整理；
- 实验性的神经网络低层控制分支。

AI **没有直接替代本人完成最终验收**。所有关键模块均通过真实 ROS 2 运行结果进行取舍。

---

## 4. 由本人重点确认的核心公式与接口

以下内容不能仅依赖 AI 输出，本人根据课程要求、源代码、ROS 2 实际运行和实验结果进行了确认：

### 4.1 四旋翼动力学

- 单电机推力：
  \[
  F_i = k_F \omega_i^2
  \]
- 电机存在一阶响应，而不是 RPM 指令瞬时到达。
- X 型四旋翼通过四电机推力差产生 roll / pitch / yaw 力矩。
- 平动包含重力和机体系推力到世界系的变换。
- 转动满足刚体 Euler 方程。
- 姿态使用四元数积分，并进行归一化。
- RPM、姿态、加速度等设置安全限制。

### 4.2 控制器

- 位置误差经过位置环产生期望加速度/合力。
- 期望合力转换为期望姿态。
- 姿态/角速度环产生期望力矩。
- mixer 将总推力与三轴力矩分配到四个电机。

### 4.3 ROS 2 核心接口

重点确认过：

- `/drone/motor_rpm_cmd`
- `/drone/odom`
- `/drone/imu`
- `/drone/path`
- `/drone/goal`
- `/map/obstacles`
- `/drone/reference`
- `/drone/waypoints`
- `/drone/safe_goal`
- TF：`map -> base_link`

---

## 5. AI 生成或建议中出现过的典型错误，以及如何发现

### 5.1 将附加扰动做成“存在但主任务不可见”

早期外力扰动模块虽然能单独运行，但没有在最终主任务中形成明显证据。

**发现方式：**
- 直接观察主任务；
- 检查 disturbance 是否真正改变实际轨迹和 recorder 数据。

**修正：**
- 将扰动绑定到明确任务阶段；
- 增加扰动曲线、姿态/RPM 响应和恢复评价。

---

### 5.2 局部感知错误订阅同名不同类型 topic

AI 初版尝试在同一节点中同时对 `/map/obstacles` 创建 MarkerArray 和 PointCloud2 subscription。

ROS 2 Humble 实际报错：

`create_subscription() called for existing topic name ... with incompatible type`

**发现方式：**
- 直接运行节点；
- 查看 traceback 和 RMW 错误。

**修正：**
- 根据实际 graph 只订阅有效 PointCloud2 类型。

---

### 5.3 YAML 参数文件被增量修改破坏

一次自动 patch 后 `requirement_closure.yaml` 无法解析。

**发现方式：**
- 两个节点均在 `rclpy.init()` 报 `Couldn't parse params file`。

**修正：**
- 不再继续字符串增量替换；
- 从头重写干净的 ROS 2 参数 YAML，并重新编译。

---

### 5.4 PointCloud2 QoS 不兼容

局部感知节点存活，但 `pointcloud_messages=0`。

最终发现：

- map publisher：`RELIABLE + TRANSIENT_LOCAL`
- subscriber：`BEST_EFFORT + VOLATILE`

**发现方式：**

`ros2 topic info /map/obstacles --verbose`

**修正：**
- 订阅端改为 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`；
- 最终局部点云输出非零。

---

### 5.5 神经网络离线指标好但闭环飞行失败

多版神经策略出现：

- validation MAE 较低；
- teacher-forced 指标改善；
- 但纯 AI hover 仍在约 1–2 s 内超过最大倾角。

**发现方式：**
- 坚持运行无 classical controller 的纯 AI hover；
- 不以离线 MAE 作为“成功”结论。

**处理：**
- 不继续包装失败结果；
- 明确将 E2E 分支排除出最终课程运行链。

---

## 6. 我如何验证 AI 生成代码不是“看起来能跑”

本项目使用以下验证方法：

1. `colcon build` 验证 ROS 2 包构建。
2. 检查节点是否真实存活。
3. `ros2 topic type` 检查消息类型。
4. `ros2 topic info --verbose` 检查 publisher/subscriber 和 QoS。
5. 检查 `/tf` 与 `map -> base_link`。
6. 使用独立 evaluator / recorder 输出 `summary.json`。
7. 使用 CSV 记录时间序列，而不是只观察 RViz。
8. 自动绘制位置误差、RPM、轨迹、障碍距离等图。
9. 对不同参数 profile 进行重复实验。
10. 对神经网络控制坚持进行真正的闭环 flight test。
11. Final Acceptance 将老师六项最低验收重新聚合，最终结果为 `pass=True; failed=[]`。

---

## 7. 最终项目中 AI 结果的取舍原则

最终提交遵循以下原则：

- **能被真实 ROS 2 运行和定量结果验证的功能才进入主系统。**
- 失败实验不通过修改阈值或隐藏日志伪装成成功。
- 多无人机最终不作为提交目标。
- RAPTOR-inspired E2E motor-AI 仅作为拓展探索，不进入最终运行链。
- 核心最终系统仍为：
  - 四旋翼动力学；
  - 经典闭环控制；
  - 3D A*；
  - 重庆山城静态/动态避障；
  - 多目标任务；
  - 扰动恢复；
  - 传感器闭环；
  - 参数 profile；
  - 自动记录与定量评价；
  - RViz / Python GUI 可视化。

---

## 8. 反思

AI 显著提高了工程迭代速度，尤其适合：

- 批量生成重复性较高的 ROS 2 脚手架；
- 根据报错快速定位可能原因；
- 生成审计和自动化测试代码；
- 对复杂实验结果进行结构化复盘。

但本项目也证明了 AI 的局限：

1. AI 给出的代码并不天然正确，ROS 2 message type、QoS、参数文件和 launch 时序等问题必须实际运行才能发现。
2. 离线机器学习指标不代表闭环控制稳定。
3. AI 很容易在已经失败的技术方向上继续提出“再试一版”的方案，项目负责人必须根据时间成本和实际收益决定何时停止。
4. 对控制系统而言，物理规律、接口一致性、闭环实验和可重复证据比“代码看起来复杂”更重要。

因此，本项目中 AI 的最终价值不是替代本人完成课程设计，而是作为高频工程辅助工具；最终技术判断和验收结论仍以本人运行、观察和定量实验为准。
