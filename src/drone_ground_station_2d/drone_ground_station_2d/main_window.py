from __future__ import annotations

import math
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QListWidget, QMainWindow, QMessageBox,
    QPushButton, QSizePolicy, QSplitter, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from .models import GroundSnapshot, VehicleState
from .ros_bridge import GoalCommand, GroundStationBridge
from .session_logger import SessionLogger
from .widgets import AMBER, CYAN, GREEN, MUTED, PURPLE, RED, TEXT, MiniChart, MissionMapWidget, RpmWidget, StatusChip


STYLE = """
QMainWindow, QWidget { background:#0d131d; color:#dfe7f1; font-family: DejaVu Sans; font-size:10pt; }
QGroupBox { border:1px solid #2a3546; border-radius:8px; margin-top:10px; padding-top:9px; font-weight:700; background:#121a26; }
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; color:#9fb0c5; }
QPushButton { background:#1e293b; border:1px solid #3a475b; border-radius:6px; padding:7px 11px; font-weight:600; }
QPushButton:hover { background:#263449; border-color:#38bdf8; }
QPushButton:pressed { background:#0f172a; }
QPushButton#primary { background:#0284c7; border-color:#38bdf8; color:white; }
QPushButton#danger { background:#7f1d1d; border-color:#fb7185; }
QComboBox, QDoubleSpinBox { background:#111827; border:1px solid #334155; border-radius:5px; padding:5px; min-height:22px; }
QTableWidget { background:#111827; alternate-background-color:#151f2e; gridline-color:#273449; border:1px solid #2b3545; }
QHeaderView::section { background:#1b2636; color:#cbd5e1; padding:5px; border:0; border-right:1px solid #2b3545; }
QListWidget { background:#0f1722; border:1px solid #2b3545; border-radius:6px; }
QTabWidget::pane { border:1px solid #2b3545; }
QTabBar::tab { background:#172131; padding:7px 14px; }
QTabBar::tab:selected { background:#0e7490; }
QSplitter::handle { background:#263244; }
"""


class MainWindow(QMainWindow):
    def __init__(self, bridge: GroundStationBridge) -> None:
        super().__init__()
        self.bridge = bridge
        self.snapshot = bridge.snapshot()
        self.logger = SessionLogger(Path.home() / 'drone_task/your_drone_sim_ws/results/ground_station_2d')
        self.last_selected = 'single'
        self.last_event_key = None
        self.last_log_sample = 0.0
        self.setWindowTitle('ROS 2 Quadrotor Mission Control — Advanced 2D Ground Station')
        self.resize(1540, 940)
        self.setMinimumSize(1180, 720)
        self.setStyleSheet(STYLE)
        self._build_ui()
        self.timer = QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(100)

    def _build_ui(self) -> None:
        root = QWidget(); self.setCentralWidget(root)
        layout = QVBoxLayout(root); layout.setContentsMargins(8,8,8,8); layout.setSpacing(7)
        top = QFrame(); top_layout = QHBoxLayout(top); top_layout.setContentsMargins(8,4,8,4)
        title = QLabel('QUADROTOR MISSION CONTROL'); title.setFont(QFont('Sans', 14, QFont.Bold)); top_layout.addWidget(title)
        self.scenario_label = QLabel('WAITING'); self.scenario_label.setStyleSheet('color:#38bdf8; font-weight:700; padding:4px 12px; background:#132333; border-radius:8px;'); top_layout.addWidget(self.scenario_label)
        top_layout.addStretch(1)
        self.single_chip, self.uav1_chip, self.uav2_chip = StatusChip(), StatusChip(), StatusChip()
        for label, chip in [('DRONE', self.single_chip), ('UAV1', self.uav1_chip), ('UAV2', self.uav2_chip)]:
            top_layout.addWidget(QLabel(label)); top_layout.addWidget(chip)
        self.record_chip = StatusChip('IDLE'); self.record_chip.set_state('IDLE', warning=True); top_layout.addWidget(self.record_chip)
        self.clock_label = QLabel('--:--:--'); self.clock_label.setFont(QFont('Monospace', 10, QFont.Bold)); top_layout.addWidget(self.clock_label)
        layout.addWidget(top)

        main_split = QSplitter(Qt.Horizontal)
        left = QWidget(); left_layout = QVBoxLayout(left); left_layout.setContentsMargins(0,0,0,0)
        map_group = QGroupBox('XY Mission Map — double-click to choose goal; wheel zoom; drag pan')
        map_layout = QVBoxLayout(map_group); self.map_widget = MissionMapWidget(); self.map_widget.targetPicked.connect(self._map_target); map_layout.addWidget(self.map_widget)
        map_tools = QHBoxLayout();
        self.fit_button = QPushButton('Fit All'); self.fit_button.clicked.connect(self.map_widget.request_auto_fit); map_tools.addWidget(self.fit_button)
        self.clear_tracks_button = QPushButton('Clear Tracks'); self.clear_tracks_button.clicked.connect(self.bridge.clear_tracks); map_tools.addWidget(self.clear_tracks_button)
        map_tools.addStretch(1); self.map_hint = QLabel('Selected target: —'); self.map_hint.setStyleSheet('color:#93a4b8'); map_tools.addWidget(self.map_hint)
        map_layout.addLayout(map_tools); left_layout.addWidget(map_group, 5)

        charts = QSplitter(Qt.Horizontal)
        self.altitude_chart = MiniChart('Altitude', 'm', CYAN)
        self.speed_chart = MiniChart('Speed', 'm/s', GREEN)
        self.error_chart = MiniChart('Goal error', 'm', AMBER)
        charts.addWidget(self.altitude_chart); charts.addWidget(self.speed_chart); charts.addWidget(self.error_chart)
        left_layout.addWidget(charts, 2)
        main_split.addWidget(left)

        right = QWidget(); right_layout = QVBoxLayout(right); right_layout.setContentsMargins(0,0,0,0)
        fleet_box = QGroupBox('Fleet Overview'); fleet_layout = QVBoxLayout(fleet_box)
        self.fleet = QTableWidget(3, 8); self.fleet.setHorizontalHeaderLabels(['Vehicle','Link','Mode','X','Y','Z','Speed','Error']); self.fleet.verticalHeader().setVisible(False); self.fleet.setAlternatingRowColors(True)
        self.fleet.setMaximumHeight(145); self.fleet.horizontalHeader().setStretchLastSection(True); fleet_layout.addWidget(self.fleet); right_layout.addWidget(fleet_box)

        tabs = QTabWidget(); right_layout.addWidget(tabs, 1)
        telemetry_tab = QWidget(); telemetry_layout = QVBoxLayout(telemetry_tab)
        selection = QHBoxLayout(); selection.addWidget(QLabel('Vehicle'))
        self.vehicle_combo = QComboBox(); self.vehicle_combo.addItem('Single UAV (/drone)', 'single'); self.vehicle_combo.addItem('UAV 1 (/uav1)', 'uav1'); self.vehicle_combo.addItem('UAV 2 (/uav2)', 'uav2'); self.vehicle_combo.currentIndexChanged.connect(self._selection_changed); selection.addWidget(self.vehicle_combo, 1); telemetry_layout.addLayout(selection)
        self.telemetry_labels: Dict[str, QLabel] = {}
        form = QFormLayout()
        for key, name in [('position','Position [m]'),('velocity','Velocity [m/s]'),('attitude','Roll / Pitch / Yaw [deg]'),('angular','Angular rate [rad/s]'),('goal','Current goal [m]'),('error','Goal error [m]'),('link_age','Odom age [s]')]:
            label = QLabel('N/A'); label.setFont(QFont('Monospace', 9, QFont.Bold)); self.telemetry_labels[key] = label; form.addRow(name, label)
        telemetry_layout.addLayout(form)
        self.rpm = RpmWidget(); telemetry_layout.addWidget(self.rpm)
        tabs.addTab(telemetry_tab, 'Telemetry')

        mission_tab = QWidget(); mission_layout = QVBoxLayout(mission_tab)
        route_row = QHBoxLayout(); route_row.addWidget(QLabel('Goal route'))
        self.route_combo = QComboBox(); self.route_combo.addItem('Direct controller /drone/goal', 'direct'); self.route_combo.addItem('3D planner /drone/goal_raw', 'planner'); self.route_combo.addItem('Dynamic supervisor /drone/goal_nominal', 'dynamic'); route_row.addWidget(self.route_combo, 1); mission_layout.addLayout(route_row)
        goal_form = QFormLayout()
        self.goal_x = self._spin(-500,500,2.0); self.goal_y = self._spin(-500,500,1.0); self.goal_z = self._spin(0,100,1.5); self.goal_yaw = self._spin(-180,180,0.0)
        for text, widget in [('X [m]',self.goal_x),('Y [m]',self.goal_y),('Z [m]',self.goal_z),('Yaw [deg]',self.goal_yaw)]: goal_form.addRow(text, widget)
        mission_layout.addLayout(goal_form)
        self.send_button = QPushButton('SEND GOAL'); self.send_button.setObjectName('primary'); self.send_button.clicked.connect(self.send_goal); mission_layout.addWidget(self.send_button)
        row = QHBoxLayout();
        for text, handler in [('Takeoff',self.takeoff),('Hold',self.hold),('Use Current',self.use_current)]:
            b=QPushButton(text); b.clicked.connect(handler); row.addWidget(b)
        mission_layout.addLayout(row)
        self.land_button = QPushButton('Controlled Land'); self.land_button.setObjectName('danger'); self.land_button.clicked.connect(self.land); mission_layout.addWidget(self.land_button)
        note = QLabel('For UAV1/UAV2, commands use direct goal topics. Do not override goals during the automatic deconfliction experiment unless deliberately testing manual intervention.'); note.setWordWrap(True); note.setStyleSheet('color:#9aa8bb'); mission_layout.addWidget(note); mission_layout.addStretch(1)
        tabs.addTab(mission_tab, 'Mission')

        system_tab = QWidget(); system_layout = QFormLayout(system_tab)
        self.system_labels = {}
        for key, name in [('planner','Planner'),('trajectory','Trajectory'),('replanning','Replanning'),('dynamic','Dynamic obstacle'),('coordinator','Coordinator'),('distance','Inter-UAV distance'),('predicted','Predicted minimum'),('safety','Safety')]:
            label=QLabel('N/A'); label.setWordWrap(True); label.setFont(QFont('Monospace',8)); self.system_labels[key]=label; system_layout.addRow(name,label)
        tabs.addTab(system_tab, 'Algorithms')

        record_box = QGroupBox('Experiment Record'); record_layout = QHBoxLayout(record_box)
        self.record_button = QPushButton('Start Recording'); self.record_button.clicked.connect(self.toggle_recording); record_layout.addWidget(self.record_button)
        self.record_path = QLabel('Not recording'); self.record_path.setWordWrap(True); self.record_path.setStyleSheet('color:#93a4b8'); record_layout.addWidget(self.record_path,1); right_layout.addWidget(record_box)
        main_split.addWidget(right); main_split.setSizes([1060, 455]); layout.addWidget(main_split, 1)

        event_box = QGroupBox('Mission Event Timeline'); event_layout = QVBoxLayout(event_box)
        self.events = QListWidget(); self.events.setMaximumHeight(112); event_layout.addWidget(self.events); layout.addWidget(event_box)
        self.statusBar().showMessage('Ground station ready. Start a ROS 2 simulation.')

    @staticmethod
    def _spin(low, high, value):
        w=QDoubleSpinBox(); w.setRange(low,high); w.setDecimals(3); w.setSingleStep(0.1); w.setValue(value); return w

    def _selection_changed(self) -> None:
        key = self.selected_key()
        if key != self.last_selected:
            self.last_selected = key
            self.altitude_chart.clear(); self.speed_chart.clear(); self.error_chart.clear()
        self.route_combo.setEnabled(key == 'single')

    def selected_key(self) -> str: return str(self.vehicle_combo.currentData())
    def selected_state(self) -> VehicleState: return self.snapshot.vehicles[self.selected_key()]

    def _map_target(self, x: float, y: float) -> None:
        self.goal_x.setValue(x); self.goal_y.setValue(y); self.map_hint.setText(f'Selected target: X={x:.2f}, Y={y:.2f}')

    def send_goal(self) -> None:
        key=self.selected_key(); route=str(self.route_combo.currentData()) if key=='single' else 'direct'
        self.bridge.enqueue_goal(GoalCommand(key,route,self.goal_x.value(),self.goal_y.value(),self.goal_z.value(),self.goal_yaw.value()))
        self.statusBar().showMessage(f'Goal queued: {key}/{route}')

    def use_current(self) -> None:
        s=self.selected_state(); self.goal_x.setValue(s.position[0]); self.goal_y.setValue(s.position[1]); self.goal_z.setValue(s.position[2]); self.goal_yaw.setValue(s.attitude_deg[2])

    def takeoff(self) -> None:
        s=self.selected_state(); self.bridge.enqueue_goal(GoalCommand(s.key,'direct',s.position[0],s.position[1],max(1.5,self.goal_z.value()),s.attitude_deg[2]))

    def hold(self) -> None:
        s=self.selected_state(); self.bridge.enqueue_goal(GoalCommand(s.key,'direct',s.position[0],s.position[1],s.position[2],s.attitude_deg[2]))

    def land(self) -> None:
        s=self.selected_state()
        if QMessageBox.question(self,'Controlled landing',f'Send a 0.05 m altitude goal to {s.label}?',QMessageBox.Yes|QMessageBox.No)==QMessageBox.Yes:
            self.bridge.enqueue_goal(GoalCommand(s.key,'direct',s.position[0],s.position[1],0.05,s.attitude_deg[2]))

    def toggle_recording(self) -> None:
        if not self.logger.active:
            directory=self.logger.start(); self.record_button.setText('Stop Recording'); self.record_path.setText(str(directory)); self.record_chip.set_state('REC', ok=True); self._event('RECORDING STARTED')
        else:
            directory=self.logger.stop(self.snapshot); self.record_button.setText('Start Recording'); self.record_path.setText(f'Saved: {directory}'); self.record_chip.set_state('IDLE', warning=True); self._event('RECORDING SAVED')

    def _event(self, text: str) -> None:
        stamp=datetime.now().strftime('%H:%M:%S.%f')[:-3]; item=f'{stamp}  {text}'; self.events.insertItem(0,item)
        while self.events.count()>80: self.events.takeItem(self.events.count()-1)
        self.logger.add_event(text)

    @staticmethod
    def _vec(values) -> str: return '(' + ', '.join(f'{float(v): .3f}' for v in values) + ')'

    def refresh(self) -> None:
        self.snapshot=self.bridge.snapshot(); now=self.snapshot.monotonic_time; self.clock_label.setText(datetime.now().strftime('%H:%M:%S'))
        self.scenario_label.setText(self.snapshot.scenario())
        for chip,key in [(self.single_chip,'single'),(self.uav1_chip,'uav1'),(self.uav2_chip,'uav2')]:
            online=self.snapshot.vehicles[key].connected(now); chip.set_state('ONLINE' if online else 'OFFLINE',ok=online)
        for text in self.bridge.drain_events(): self._event(text)
        self.map_widget.set_snapshot(self.snapshot)
        for row,key in enumerate(['single','uav1','uav2']):
            s=self.snapshot.vehicles[key]; online=s.connected(now); error=s.goal_error(); mode='OFFLINE'
            if online:
                if s.position[2] < 0.08 and s.speed() < 0.08: mode='GROUND'
                elif error is not None and error < 0.12 and s.speed() < 0.12: mode='HOLD'
                else: mode='TRACKING'
            values=[s.label,'ONLINE' if online else 'OFFLINE',mode,f'{s.position[0]:.2f}',f'{s.position[1]:.2f}',f'{s.position[2]:.2f}',f'{s.speed():.2f}','—' if error is None else f'{error:.2f}']
            for col,value in enumerate(values):
                item=QTableWidgetItem(value); item.setTextAlignment(Qt.AlignCenter)
                if col==1: item.setForeground(QColor('#4ade80') if online else QColor('#fb7185'))
                self.fleet.setItem(row,col,item)
        s=self.selected_state(); error=s.goal_error(); age=now-s.last_odom_monotonic if s.last_odom_monotonic else math.inf
        self.telemetry_labels['position'].setText(self._vec(s.position)); self.telemetry_labels['velocity'].setText(self._vec(s.velocity)); self.telemetry_labels['attitude'].setText(self._vec(s.attitude_deg)); self.telemetry_labels['angular'].setText(self._vec(s.angular_velocity)); self.telemetry_labels['goal'].setText(self._vec(s.goal) if s.has_goal else 'N/A'); self.telemetry_labels['error'].setText('N/A' if error is None else f'{error:.4f}'); self.telemetry_labels['link_age'].setText('N/A' if not math.isfinite(age) else f'{age:.3f}')
        self.rpm.set_values(s.rpm); self.altitude_chart.append(s.position[2] if s.connected(now) else None); self.speed_chart.append(s.speed() if s.connected(now) else None); self.error_chart.append(error)
        sys=self.snapshot.system; dyn=self.snapshot.dynamic_obstacle
        self.system_labels['planner'].setText(str(sys.planner_status.get('state','N/A'))); self.system_labels['trajectory'].setText(str(sys.trajectory_status.get('state','N/A'))); self.system_labels['replanning'].setText(str(sys.replanning_status.get('state','N/A'))); self.system_labels['dynamic'].setText(f'{dyn.phase}  pos={dyn.position}' if dyn.position else dyn.phase); self.system_labels['coordinator'].setText(sys.coordinator_state); self.system_labels['distance'].setText('N/A' if sys.current_inter_uav_distance_m is None else f'{sys.current_inter_uav_distance_m:.3f} m'); self.system_labels['predicted'].setText('N/A' if sys.predicted_inter_uav_distance_m is None else f'{sys.predicted_inter_uav_distance_m:.3f} m'); self.system_labels['safety'].setText('ACTIVE' if sys.safety_active else 'CLEAR'); self.system_labels['safety'].setStyleSheet('color:#fb7185;font-weight:700' if sys.safety_active else 'color:#4ade80;font-weight:700')
        if self.logger.active and now-self.last_log_sample>=0.1: self.logger.append(self.snapshot); self.last_log_sample=now

    def closeEvent(self,event) -> None:
        if self.logger.active: self.logger.stop(self.snapshot)
        event.accept()
