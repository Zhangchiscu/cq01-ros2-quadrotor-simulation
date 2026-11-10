#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import threading
from copy import deepcopy
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, Float64, String


class DashboardDataNode(Node):
    """Read-only ROS node for the CQ01 bonus dashboard.

    This node deliberately creates no publishers and therefore cannot send a
    target, switch the controller, or apply a disturbance.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v9_bonus_dashboard')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._lock = threading.Lock()
        self._data: Dict[str, Any] = {
            'position': None,
            'velocity': None,
            'attitude_deg': None,
            'goal': None,
            'rpm': None,
            'clearance_m': None,
            'applied_force_n': None,
            'statuses': {},
        }

        self.create_subscription(
            Odometry, '/drone/odom', self._odom_callback, 30
        )
        self.create_subscription(
            PoseStamped, '/drone/goal', self._goal_callback, 10
        )
        self.create_subscription(
            Float32MultiArray,
            '/drone/motor_rpm',
            self._rpm_callback,
            20,
        )
        self.create_subscription(
            Float64,
            '/cq01_v9/course/body_clearance',
            self._clearance_callback,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            '/drone/external_wrench_applied',
            self._wrench_callback,
            20,
        )

        status_topics = {
            'ground_station': '/cq01_v9/ground_station/status',
            'planner': '/drone/planner_3d/status',
            'route': '/cq01_v9/route/status',
            'sensors': '/cq01_v9/sensors/status',
            'sensor_chain': '/cq01_v9/sensor_chain/status',
            'low_strong': '/cq01_v9/low_strong/status',
            'low_replan': '/cq01_v9/low_replan/status',
            'climb': '/cq01_v9/climb/status',
            'h14': '/cq01_v9/h14/status',
            'dynamic': '/cq01_v9/dynamic/status',
            'pattern': '/cq01_v9/pattern/status',
            'pattern_result': '/cq01_v9/pattern/result',
            'integrated': '/cq01_v9/integrated/status',
            'recorder': '/cq01_v9/bonus_recorder/status',
            'profile': '/cq01_v9/profile/status',
        }
        for key, topic in status_topics.items():
            self.create_subscription(
                String,
                topic,
                lambda message, status_key=key: self._status_callback(
                    status_key, message
                ),
                transient,
            )

        self.get_logger().warning(
            'CQ01 bonus dashboard is READ-ONLY: no ROS publishers created.'
        )

    @staticmethod
    def _quaternion_to_euler_deg(
        x: float, y: float, z: float, w: float
    ) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(
            math.pi / 2.0, sinp
        ) if abs(sinp) >= 1.0 else math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return tuple(math.degrees(value) for value in (roll, pitch, yaw))

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        q = message.pose.pose.orientation
        attitude = self._quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
        with self._lock:
            self._data['position'] = (position.x, position.y, position.z)
            self._data['velocity'] = (velocity.x, velocity.y, velocity.z)
            self._data['attitude_deg'] = attitude

    def _goal_callback(self, message: PoseStamped) -> None:
        point = message.pose.position
        with self._lock:
            self._data['goal'] = (point.x, point.y, point.z)

    def _rpm_callback(self, message: Float32MultiArray) -> None:
        with self._lock:
            self._data['rpm'] = tuple(float(v) for v in message.data[:4])

    def _clearance_callback(self, message: Float64) -> None:
        with self._lock:
            self._data['clearance_m'] = float(message.data)

    def _wrench_callback(self, message: WrenchStamped) -> None:
        force = message.wrench.force
        magnitude = math.sqrt(
            force.x * force.x + force.y * force.y + force.z * force.z
        )
        with self._lock:
            self._data['applied_force_n'] = magnitude

    def _status_callback(self, key: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {'raw': message.data}
        with self._lock:
            self._data['statuses'][key] = payload

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return deepcopy(self._data)


class BonusDashboard:
    def __init__(self, node: DashboardDataNode) -> None:
        try:
            import tkinter as tk
            from tkinter import ttk
        except ImportError as exc:
            raise RuntimeError(
                'tkinter is unavailable. Install it with: '
                'sudo apt install python3-tk'
            ) from exc

        self.tk = tk
        self.ttk = ttk
        self.node = node
        self.root = tk.Tk()
        self.root.title('CQ01 重庆山城全功能地面站（只读）')
        self.root.geometry('1000x720')
        self.root.minsize(900, 620)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.variables: Dict[str, tk.StringVar] = {}
        self._build_ui()
        self.root.after(200, self._refresh)

    def _make_var(self, name: str, default: str = '--'):
        var = self.tk.StringVar(value=default)
        self.variables[name] = var
        return var

    def _build_ui(self) -> None:
        ttk = self.ttk
        header = ttk.Frame(self.root, padding=10)
        header.pack(fill='x')
        ttk.Label(
            header,
            text='CQ01 重庆山城全功能闭环地面站',
            font=('Sans', 18, 'bold'),
        ).pack(side='left')
        ttk.Label(
            header,
            text='只读｜无目标发布器｜无控制权限',
            font=('Sans', 11, 'bold'),
        ).pack(side='right')

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=10, pady=(0, 10))

        live = ttk.Frame(notebook, padding=12)
        bonus = ttk.Frame(notebook, padding=12)
        reference = ttk.Frame(notebook, padding=12)
        notebook.add(live, text='实时监控')
        notebook.add(bonus, text='加分项状态')
        notebook.add(reference, text='参考架构')

        self._build_live_tab(live)
        self._build_bonus_tab(bonus)
        self._build_reference_tab(reference)

        footer = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        footer.pack(fill='x')
        ttk.Label(footer, textvariable=self._make_var('footer')).pack(
            side='left'
        )
        ttk.Button(footer, text='关闭窗口', command=self.close).pack(
            side='right'
        )

    def _add_value_row(self, parent, row: int, label: str, key: str) -> None:
        self.ttk.Label(parent, text=label).grid(
            row=row, column=0, sticky='w', padx=6, pady=5
        )
        self.ttk.Label(
            parent,
            textvariable=self._make_var(key),
            font=('Monospace', 11),
        ).grid(row=row, column=1, sticky='w', padx=6, pady=5)

    def _build_live_tab(self, parent) -> None:
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

        flight = self.ttk.LabelFrame(parent, text='飞行状态', padding=10)
        flight.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)
        values = [
            ('任务阶段', 'stage'),
            ('当前位置 x/y/z (m)', 'position'),
            ('速度 vx/vy/vz (m/s)', 'velocity'),
            ('姿态：横滚／俯仰／偏航（度）', 'attitude'),
            ('当前目标 x/y/z (m)', 'goal'),
            ('位置误差 (m)', 'position_error'),
        ]
        for row, (label, key) in enumerate(values):
            self._add_value_row(flight, row, label, key)

        system = self.ttk.LabelFrame(parent, text='控制与安全', padding=10)
        system.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)
        values = [
            ('四电机 RPM', 'rpm'),
            ('最大 RPM', 'max_rpm'),
            ('机体净空 (m)', 'clearance'),
            ('外力幅值 (N)', 'force'),
            ('A* 状态', 'planner_state'),
            ('A* 规划耗时 (ms)', 'planning_time'),
        ]
        for row, (label, key) in enumerate(values):
            self._add_value_row(system, row, label, key)

        modules = self.ttk.LabelFrame(parent, text='重庆闭环模块', padding=10)
        modules.grid(
            row=1, column=0, columnspan=2, sticky='nsew', padx=5, pady=5
        )
        modules.columnconfigure(1, weight=1)
        module_rows = [
            ('P0–P15 路线', 'route_state'),
            ('低空强风', 'low_strong_state'),
            ('低空扰动重规划', 'low_replan_state'),
            ('爬升扰动', 'climb_state'),
            ('高空强风重规划', 'h14_state'),
            ('动态障碍', 'dynamic_state'),
            ('圆形/八字', 'pattern_state'),
            ('传感器闭环', 'sensor_chain_state'),
            ('参数配置', 'profile_state'),
            ('统一记录器', 'recorder_state'),
            ('最终结果', 'final_result'),
        ]
        for row, (label, key) in enumerate(module_rows):
            self._add_value_row(modules, row, label, key)

    def _build_bonus_tab(self, parent) -> None:
        rows = [
            ('1 参数化质量/惯量/电机/控制增益', 'nominal/heavy 已接入同一重庆完整闭环'),
            ('2 外力扰动', '已进入重庆主闭环'),
            ('3 传感器噪声与简化 IMU', 'nominal noisy odom 已固定进入重庆反馈闭环；简化 IMU 同步输出'),
            ('4 点云/体素地图/局部感知', '已进入重庆主闭环'),
            ('6 航点/圆形/八字', '已进入重庆主闭环'),
            ('7 一键测试与自动记录', '本阶段接入统一记录器'),
            ('8 Python 图形化地面站', '本窗口；严格只读'),
            ('9 pengyu_sim / MARSIM 对比', '见“参考架构”页面'),
            ('10 其他创新', '山城高差、动态障碍、高空重规划、全路线净空'),
        ]
        parent.columnconfigure(1, weight=1)
        for index, (name, status) in enumerate(rows):
            self.ttk.Label(
                parent, text=name, font=('Sans', 11, 'bold')
            ).grid(row=index, column=0, sticky='nw', padx=8, pady=8)
            self.ttk.Label(
                parent,
                text=status,
                wraplength=560,
                justify='left',
            ).grid(row=index, column=1, sticky='nw', padx=8, pady=8)

        self.ttk.Separator(parent).grid(
            row=len(rows), column=0, columnspan=2, sticky='ew', pady=12
        )
        self.ttk.Label(
            parent,
            text=(
                '说明：参数化和带噪控制将使用同一重庆场景的独立配置运行，'
                '不在单次飞行中途切换动力学或里程计源。'
            ),
            wraplength=820,
            justify='left',
        ).grid(row=len(rows) + 1, column=0, columnspan=2, sticky='w')

    def _build_reference_tab(self, parent) -> None:
        text = self.tk.Text(parent, wrap='word', font=('Sans', 11))
        text.pack(fill='both', expand=True)
        content = (
            '本项目与参考仓库的关系\n\n'
            '1. 本项目\n'
            '• 自主实现六自由度四旋翼动力学、级联控制器、三维 A*、'
            '任务管理、扰动管理、动态障碍和自动评价。\n'
            '• 最终验证环境为重庆山城高差、密集建筑、桥下和峡谷场景。\n\n'
            '2. pengyu_sim\n'
            '• 仅用于理解 ROS 2 无人机仿真工程的节点组织、话题接口和'
            '启动方式。\n'
            '• 未直接封装其动力学、控制器或任务逻辑作为本项目结果。\n\n'
            '3. MARSIM\n'
            '• 主要参考复杂三维点云环境和无人机仿真场景表达思路。\n'
            '• 本项目仍使用自主重庆场景、体素适配和三维 A* 闭环。\n\n'
            '4. 差异化创新\n'
            '• 在同一连续任务中组合低空阵风、高空强风重规划、持续动态'
            '障碍、快速拉升、圆形与八字轨迹。\n'
            '• 使用统一评价器记录净空、误差、RPM、规划耗时和最终 PASS/FAIL。\n'
        )
        text.insert('1.0', content)
        text.configure(state='disabled')

    @staticmethod
    def _fmt_vector(value: Optional[tuple], digits: int = 2) -> str:
        if value is None:
            return '--'
        return ' / '.join(f'{float(item):.{digits}f}' for item in value)

    @staticmethod
    def _status_state(statuses: Dict[str, Any], key: str) -> str:
        payload = statuses.get(key, {})
        if not isinstance(payload, dict):
            return '--'
        state = payload.get('state')
        if state is None and 'pass' in payload:
            state = 'PASS' if payload.get('pass') else 'FAIL'
        return str(state) if state is not None else '--'

    def _refresh(self) -> None:
        data = self.node.snapshot()
        statuses = data.get('statuses', {})
        ground = statuses.get('ground_station', {})
        planner = statuses.get('planner', {})
        position = data.get('position')
        goal = data.get('goal')

        error = None
        if position is not None and goal is not None:
            error = math.dist(position, goal)

        self.variables['stage'].set(str(ground.get('stage', '--')))
        self.variables['position'].set(self._fmt_vector(position))
        self.variables['velocity'].set(self._fmt_vector(data.get('velocity')))
        self.variables['attitude'].set(
            self._fmt_vector(data.get('attitude_deg'))
        )
        self.variables['goal'].set(self._fmt_vector(goal))
        self.variables['position_error'].set(
            '--' if error is None else f'{error:.3f}'
        )

        rpm = data.get('rpm')
        self.variables['rpm'].set(self._fmt_vector(rpm, digits=0))
        self.variables['max_rpm'].set(
            '--' if not rpm else f'{max(rpm):.0f}'
        )
        clearance = data.get('clearance_m')
        self.variables['clearance'].set(
            '--' if clearance is None else f'{clearance:.3f}'
        )
        force = data.get('applied_force_n')
        self.variables['force'].set('--' if force is None else f'{force:.3f}')
        self.variables['planner_state'].set(str(planner.get('state', '--')))
        planning_ms = planner.get('planning_time_ms')
        self.variables['planning_time'].set(
            '--' if planning_ms is None else f'{float(planning_ms):.2f}'
        )

        for key, status_key in (
            ('route_state', 'route'),
            ('low_strong_state', 'low_strong'),
            ('low_replan_state', 'low_replan'),
            ('climb_state', 'climb'),
            ('h14_state', 'h14'),
            ('dynamic_state', 'dynamic'),
            ('pattern_state', 'pattern'),
            ('sensor_chain_state', 'sensor_chain'),
            ('profile_state', 'profile'),
            ('recorder_state', 'recorder'),
        ):
            self.variables[key].set(self._status_state(statuses, status_key))

        pattern_result = statuses.get('pattern_result', {})
        integrated = statuses.get('integrated', {})
        h14 = statuses.get('h14', {})
        profile = statuses.get('profile', {})
        sensor_chain = statuses.get('sensor_chain', {})

        integrated_done = (
            isinstance(integrated, dict)
            and integrated.get('state')
            == 'integrated_completed'
        )
        strict_checks = [
            pattern_result.get('pass') is True,
            h14.get('pass_result') is True,
            profile.get('pass') is True,
        ]
        if sensor_chain:
            strict_checks.append(
                sensor_chain.get('pass') is True
            )

        if integrated_done:
            final_text = (
                'PASS'
                if all(strict_checks)
                else 'FAIL'
            )
            final_text += ' | integrated_completed'
        else:
            final_text = '--'
            if (
                isinstance(integrated, dict)
                and integrated.get('state')
            ):
                final_text += (
                    f" | {integrated.get('state')}"
                )
        self.variables['final_result'].set(final_text)

        sensor = statuses.get('sensors', {})
        metrics = sensor.get('metrics', {}) if isinstance(sensor, dict) else {}
        nominal_rmse = metrics.get('nominal_position_rmse_m')
        self.variables['footer'].set(
            '标称传感器 RMSE：'
            + ('--' if nominal_rmse is None else f'{float(nominal_rmse):.3f} m')
            + '｜图形界面不发布 ROS2 话题'
        )
        self.root.after(200, self._refresh)

    def close(self) -> None:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardDataNode()
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spin_thread.start()
    try:
        dashboard = BonusDashboard(node)
        dashboard.run()
    except Exception as exc:
        node.get_logger().error(f'Bonus dashboard failed: {exc}')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
