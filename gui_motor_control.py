"""
gui_motor_control.py - PyQt5 기반 EtherCAT 모터 제어 GUI

기능:
1. 초기 설정: 어댑터, 모터 수, 사이클 타임, 축, 속도, 가감속
2. 런타임 제어: 원점 설정, 이동, Cross Coupling 게인 조절
3. 모니터링: 상태 테이블, 실시간 위치 그래프, 동기화 오차 그래프

사용법:
    python gui_motor_control.py
"""

import sys
import time
from collections import deque
from functools import partial

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFormLayout, QGroupBox, QComboBox, QSpinBox, QDoubleSpinBox,
    QPushButton, QRadioButton, QButtonGroup, QCheckBox, QSlider,
    QLabel, QTableWidget, QTableWidgetItem, QScrollArea, QMessageBox,
    QHeaderView, QSplitter, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont

import pyqtgraph as pg
import numpy as np

try:
    import pysoem
    PYSOEM_AVAILABLE = True
except ImportError:
    PYSOEM_AVAILABLE = False

from motor_coupling import (
    EtherCATBusCoupling,
    DEFAULT_MAX_SYNC_ERROR_MM, DEFAULT_COUPLING_GAIN
)

# --- pyqtgraph 전역 설정 ---
pg.setConfigOptions(antialias=True)

# --- 모터별 그래프 색상 ---
MOTOR_COLORS = ['#2196F3', '#F44336', '#4CAF50', '#FF9800',
                '#9C27B0', '#00BCD4', '#795548', '#607D8B']


def decode_cia402_state(status_word: int) -> str:
    """CiA 402 상태 워드를 상태 이름으로 디코딩"""
    if status_word & 0x0008:
        return "Fault"
    if (status_word & 0x006F) == 0x0027:
        return "Operation Enabled"
    if (status_word & 0x006F) == 0x0023:
        return "Switched On"
    if (status_word & 0x006F) == 0x0021:
        return "Ready to Switch On"
    if (status_word & 0x004F) == 0x0040:
        return "Switch On Disabled"
    return f"Unknown (0x{status_word:04X})"


# ============================================================
# Panel 1: 모터별 초기 설정 위젯
# ============================================================
class MotorSettingsWidget(QGroupBox):
    """개별 모터의 초기 설정 (축, 속도, 가감속)"""

    def __init__(self, motor_index: int, parent=None):
        super().__init__(f"Motor {motor_index}", parent)
        self._index = motor_index
        self._setup_ui()

    def _setup_ui(self):
        layout = QFormLayout()
        layout.setContentsMargins(8, 12, 8, 8)

        # 축 선택
        axis_layout = QHBoxLayout()
        self._btn_group = QButtonGroup(self)
        self._radio_x = QRadioButton("X")
        self._radio_z = QRadioButton("Z")
        self._radio_z.setChecked(True)
        self._btn_group.addButton(self._radio_x)
        self._btn_group.addButton(self._radio_z)
        axis_layout.addWidget(self._radio_x)
        axis_layout.addWidget(self._radio_z)
        axis_layout.addStretch()
        layout.addRow("Axis:", axis_layout)

        # 속도
        self._spin_velocity = QSpinBox()
        self._spin_velocity.setRange(1, 500)
        self._spin_velocity.setValue(50)
        self._spin_velocity.setSuffix(" RPM")
        layout.addRow("Velocity:", self._spin_velocity)

        # 가속도
        self._spin_accel = QSpinBox()
        self._spin_accel.setRange(1, 1000)
        self._spin_accel.setValue(50)
        self._spin_accel.setSuffix(" RPM/s")
        layout.addRow("Accel:", self._spin_accel)

        # 감속도
        self._spin_decel = QSpinBox()
        self._spin_decel.setRange(1, 1000)
        self._spin_decel.setValue(50)
        self._spin_decel.setSuffix(" RPM/s")
        layout.addRow("Decel:", self._spin_decel)

        self.setLayout(layout)

    def get_axis(self) -> str:
        return 'x' if self._radio_x.isChecked() else 'z'

    def get_velocity(self) -> int:
        return self._spin_velocity.value()

    def get_accel(self) -> int:
        return self._spin_accel.value()

    def get_decel(self) -> int:
        return self._spin_decel.value()


# ============================================================
# Panel 1: 초기 설정 패널
# ============================================================
class InitialSettingsPanel(QGroupBox):
    """EtherCAT 버스 및 모터 초기 설정"""

    def __init__(self, on_num_motors_changed=None, parent=None):
        super().__init__("Initial Settings / 초기 설정", parent)
        self._motor_widgets = []
        self._on_num_motors_changed = on_num_motors_changed
        self._setup_ui()

    def _setup_ui(self):
        main_layout = QVBoxLayout()

        # 버스 설정
        bus_form = QFormLayout()

        self._combo_adapter = QComboBox()
        self._combo_adapter.setEditable(True)
        self._populate_adapters()
        bus_form.addRow("Adapter:", self._combo_adapter)

        self._spin_num_motors = QSpinBox()
        self._spin_num_motors.setRange(1, 8)
        self._spin_num_motors.setValue(2)
        self._spin_num_motors.valueChanged.connect(self._rebuild_motor_widgets)
        bus_form.addRow("Motors:", self._spin_num_motors)

        self._spin_cycle_time = QSpinBox()
        self._spin_cycle_time.setRange(1, 100)
        self._spin_cycle_time.setValue(10)
        self._spin_cycle_time.setSuffix(" ms")
        bus_form.addRow("Cycle Time:", self._spin_cycle_time)

        main_layout.addLayout(bus_form)

        # 모터별 설정 영역
        self._motor_area = QScrollArea()
        self._motor_area.setWidgetResizable(True)
        self._motor_container = QWidget()
        self._motor_layout = QVBoxLayout()
        self._motor_layout.setContentsMargins(0, 0, 0, 0)
        self._motor_container.setLayout(self._motor_layout)
        self._motor_area.setWidget(self._motor_container)
        main_layout.addWidget(self._motor_area)

        # 버튼
        btn_layout = QHBoxLayout()
        self.btn_start = QPushButton("▶ Start")
        self.btn_start.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }")
        self.btn_stop = QPushButton("■ Stop")
        self.btn_stop.setStyleSheet("QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }")
        self.btn_stop.setEnabled(False)
        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        main_layout.addLayout(btn_layout)

        self.setLayout(main_layout)

        # 초기 모터 위젯 생성
        self._rebuild_motor_widgets(self._spin_num_motors.value())

    def _populate_adapters(self):
        """사용 가능한 네트워크 어댑터 목록 조회"""
        self._combo_adapter.clear()
        if not PYSOEM_AVAILABLE:
            self._combo_adapter.addItem("pysoem not available")
            return
        try:
            adapters = pysoem.find_adapters()
            if not adapters:
                self._combo_adapter.addItem("No adapters found (run as admin?)")
            else:
                for adapter in adapters:
                    self._combo_adapter.addItem(f"{adapter.desc}", adapter.name)
        except Exception as e:
            self._combo_adapter.addItem(f"Error: {e}")

    def _rebuild_motor_widgets(self, n: int):
        """모터 수 변경 시 설정 위젯 재구성"""
        for w in self._motor_widgets:
            w.setParent(None)
            w.deleteLater()
        self._motor_widgets.clear()

        for i in range(n):
            w = MotorSettingsWidget(i)
            self._motor_layout.addWidget(w)
            self._motor_widgets.append(w)
        self._motor_layout.addStretch()

        if self._on_num_motors_changed:
            self._on_num_motors_changed(n)

    def get_adapter(self) -> str:
        idx = self._combo_adapter.currentIndex()
        data = self._combo_adapter.itemData(idx)
        if data:
            return data
        return self._combo_adapter.currentText()

    def get_num_motors(self) -> int:
        return self._spin_num_motors.value()

    def get_cycle_time(self) -> int:
        return self._spin_cycle_time.value()

    def get_motor_settings(self) -> list:
        """각 모터의 설정을 딕셔너리 리스트로 반환"""
        return [{
            'axis': w.get_axis(),
            'velocity': w.get_velocity(),
            'accel': w.get_accel(),
            'decel': w.get_decel()
        } for w in self._motor_widgets]


# ============================================================
# Panel 2: 모터별 런타임 제어 위젯
# ============================================================
class MotorRuntimeWidget(QGroupBox):
    """개별 모터의 런타임 제어 (원점, 이동)"""

    def __init__(self, motor_index: int, on_origin=None, on_move=None, parent=None):
        super().__init__(f"Motor {motor_index}", parent)
        self._index = motor_index
        self._on_origin = on_origin
        self._on_move = on_move
        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)

        self._btn_origin = QPushButton("Set Origin")
        self._btn_origin.clicked.connect(lambda: self._on_origin(self._index) if self._on_origin else None)
        layout.addWidget(self._btn_origin)

        self._spin_target = QDoubleSpinBox()
        self._spin_target.setRange(-9999.99, 9999.99)
        self._spin_target.setValue(-50.0)
        self._spin_target.setDecimals(2)
        self._spin_target.setSuffix(" mm")
        layout.addWidget(self._spin_target)

        self._btn_move = QPushButton("Move")
        self._btn_move.setStyleSheet("QPushButton { background-color: #2196F3; color: white; }")
        self._btn_move.clicked.connect(
            lambda: self._on_move(self._index, self._spin_target.value()) if self._on_move else None
        )
        layout.addWidget(self._btn_move)

        self.setLayout(layout)

    def get_target(self) -> float:
        return self._spin_target.value()


# ============================================================
# Panel 2: 런타임 제어 패널
# ============================================================
class RuntimeControlPanel(QGroupBox):
    """런타임 제어: 이동, Cross Coupling, 동기화 오류 리셋"""

    def __init__(self, parent=None):
        super().__init__("Runtime Control / 런타임 제어", parent)
        self._motor_widgets = []
        self._setup_ui()
        self.setEnabled(False)

    def _setup_ui(self):
        main_layout = QVBoxLayout()

        # 모터별 런타임 위젯 영역
        self._motor_area = QScrollArea()
        self._motor_area.setWidgetResizable(True)
        self._motor_container = QWidget()
        self._motor_layout = QVBoxLayout()
        self._motor_layout.setContentsMargins(0, 0, 0, 0)
        self._motor_container.setLayout(self._motor_layout)
        self._motor_area.setWidget(self._motor_container)
        self._motor_area.setMaximumHeight(150)
        main_layout.addWidget(self._motor_area)

        # Move All
        move_all_layout = QHBoxLayout()
        move_all_layout.addWidget(QLabel("All:"))
        self._spin_move_all = QDoubleSpinBox()
        self._spin_move_all.setRange(-9999.99, 9999.99)
        self._spin_move_all.setValue(-50.0)
        self._spin_move_all.setDecimals(2)
        self._spin_move_all.setSuffix(" mm")
        move_all_layout.addWidget(self._spin_move_all)
        self.btn_move_all = QPushButton("Move All")
        self.btn_move_all.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }")
        move_all_layout.addWidget(self.btn_move_all)
        main_layout.addLayout(move_all_layout)

        # Cross Coupling 설정
        coupling_group = QGroupBox("Cross Coupling")
        coupling_layout = QFormLayout()

        self.chk_coupling = QCheckBox("Enable")
        self.chk_coupling.setChecked(True)
        coupling_layout.addRow("", self.chk_coupling)

        gain_layout = QHBoxLayout()
        self.slider_gain = QSlider(Qt.Horizontal)
        self.slider_gain.setRange(0, 100)
        self.slider_gain.setValue(int(DEFAULT_COUPLING_GAIN * 100))
        self.slider_gain.setTickInterval(5)
        self.slider_gain.setTickPosition(QSlider.TicksBelow)
        gain_layout.addWidget(self.slider_gain)
        self.lbl_gain_value = QLabel(f"{DEFAULT_COUPLING_GAIN:.2f}")
        self.lbl_gain_value.setMinimumWidth(35)
        gain_layout.addWidget(self.lbl_gain_value)
        coupling_layout.addRow("Gain:", gain_layout)

        self.spin_max_sync_err = QDoubleSpinBox()
        self.spin_max_sync_err.setRange(0.01, 10.0)
        self.spin_max_sync_err.setValue(DEFAULT_MAX_SYNC_ERROR_MM)
        self.spin_max_sync_err.setDecimals(2)
        self.spin_max_sync_err.setSuffix(" mm")
        coupling_layout.addRow("Max Sync Err:", self.spin_max_sync_err)

        coupling_group.setLayout(coupling_layout)
        main_layout.addWidget(coupling_group)

        # Reset Sync Error
        self.btn_reset_sync = QPushButton("Reset Sync Error")
        self.btn_reset_sync.setStyleSheet(
            "QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 6px; }"
        )
        main_layout.addWidget(self.btn_reset_sync)

        self.setLayout(main_layout)

    def rebuild_motor_widgets(self, n: int, on_origin=None, on_move=None):
        """모터 수에 맞게 런타임 위젯 재구성"""
        for w in self._motor_widgets:
            w.setParent(None)
            w.deleteLater()
        self._motor_widgets.clear()

        for i in range(n):
            w = MotorRuntimeWidget(i, on_origin=on_origin, on_move=on_move)
            self._motor_layout.addWidget(w)
            self._motor_widgets.append(w)

    def get_move_all_target(self) -> float:
        return self._spin_move_all.value()

    def get_coupling_enabled(self) -> bool:
        return self.chk_coupling.isChecked()

    def get_coupling_gain(self) -> float:
        return self.slider_gain.value() / 100.0

    def get_max_sync_error(self) -> float:
        return self.spin_max_sync_err.value()


# ============================================================
# Panel 3: 모니터링 패널
# ============================================================
class MonitoringPanel(QGroupBox):
    """상태 테이블 + 실시간 그래프"""

    def __init__(self, parent=None):
        super().__init__("Monitoring / 모니터링", parent)
        self._setup_ui()

    def _setup_ui(self):
        main_layout = QVBoxLayout()

        # 상태 테이블
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            ["Motor", "Status", "State", "Position (mm)", "Position (pulse)", "Moving"]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setMaximumHeight(120)
        self.table.verticalHeader().setVisible(False)
        main_layout.addWidget(self.table)

        # 인디케이터 행
        indicator_layout = QHBoxLayout()
        indicator_layout.addWidget(QLabel("Position Diff:"))
        self.lbl_pos_diff = QLabel("M0-M1: 0.000 mm")
        self.lbl_pos_diff.setFont(QFont("Consolas", 10))
        indicator_layout.addWidget(self.lbl_pos_diff)
        indicator_layout.addStretch()

        self.lbl_sync_indicator = QLabel("SYNC OK")
        self.lbl_sync_indicator.setAlignment(Qt.AlignCenter)
        self.lbl_sync_indicator.setMinimumWidth(100)
        self.lbl_sync_indicator.setStyleSheet(
            "color: white; background-color: #4CAF50; padding: 4px 8px; font-weight: bold; border-radius: 3px;"
        )
        indicator_layout.addWidget(self.lbl_sync_indicator)
        main_layout.addLayout(indicator_layout)

        # 위치 그래프
        self.plot_positions = pg.PlotWidget(title="Motor Positions vs Time")
        self.plot_positions.setLabel('left', 'Position', units='mm')
        self.plot_positions.setLabel('bottom', 'Time', units='s')
        self.plot_positions.addLegend(offset=(10, 10))
        self.plot_positions.showGrid(x=True, y=True, alpha=0.3)
        main_layout.addWidget(self.plot_positions)

        # 위치 차이 그래프
        self.plot_diff = pg.PlotWidget(title="Position Difference vs Time")
        self.plot_diff.setLabel('left', 'Diff', units='mm')
        self.plot_diff.setLabel('bottom', 'Time', units='s')
        self.plot_diff.showGrid(x=True, y=True, alpha=0.3)
        main_layout.addWidget(self.plot_diff)

        self.setLayout(main_layout)

    def setup_for_motors(self, n: int):
        """모터 수에 맞게 테이블 행 설정"""
        self.table.setRowCount(n)
        for i in range(n):
            for j in range(6):
                self.table.setItem(i, j, QTableWidgetItem(""))

    def update_row(self, row: int, status_word: int, state_name: str,
                   pos_mm: float, pos_pulse: int, moving: bool):
        """모터 상태 테이블 한 행 업데이트"""
        self.table.item(row, 0).setText(f"Motor {row}")

        self.table.item(row, 1).setText(f"0x{status_word:04X}")

        state_item = self.table.item(row, 2)
        state_item.setText(state_name)
        if "Fault" in state_name:
            state_item.setBackground(QColor(255, 200, 200))
        elif state_name == "Operation Enabled":
            state_item.setBackground(QColor(200, 255, 200))
        else:
            state_item.setBackground(QColor(255, 255, 255))

        self.table.item(row, 3).setText(f"{pos_mm:.3f}")
        self.table.item(row, 4).setText(str(pos_pulse))

        moving_item = self.table.item(row, 5)
        if moving:
            moving_item.setText("Moving")
            moving_item.setBackground(QColor(200, 230, 255))
        else:
            moving_item.setText("Stopped")
            moving_item.setBackground(QColor(255, 255, 255))

    def update_sync_indicator(self, has_error: bool):
        """동기화 상태 인디케이터 업데이트"""
        if has_error:
            self.lbl_sync_indicator.setText("SYNC ERROR")
            self.lbl_sync_indicator.setStyleSheet(
                "color: white; background-color: #F44336; padding: 4px 8px; font-weight: bold; border-radius: 3px;"
            )
        else:
            self.lbl_sync_indicator.setText("SYNC OK")
            self.lbl_sync_indicator.setStyleSheet(
                "color: white; background-color: #4CAF50; padding: 4px 8px; font-weight: bold; border-radius: 3px;"
            )


# ============================================================
# 메인 윈도우
# ============================================================
class MotorControlGUI(QMainWindow):
    """EtherCAT 모터 제어 메인 GUI"""

    def __init__(self):
        super().__init__()
        self._bus = None
        self._app_state = "idle"
        self._monitor_timer = QTimer()
        self._monitor_timer.timeout.connect(self._update_monitoring)
        self._startup_timer = None
        self._startup_check_count = 0

        # 그래프 데이터
        self._plot_max_points = 300  # 30초 (100ms 간격)
        self._time_data = deque(maxlen=self._plot_max_points)
        self._position_data = {}
        self._diff_data = deque(maxlen=self._plot_max_points)
        self._plot_start_time = None
        self._position_curves = {}
        self._diff_curve = None
        self._threshold_line = None

        self._setup_ui()
        self._connect_signals()
        self._update_panel_states()

    def _setup_ui(self):
        self.setWindowTitle("EtherCAT Motor Control")
        self.setMinimumSize(1200, 800)
        self.resize(1400, 900)

        central = QWidget()
        self.setCentralWidget(central)

        # 좌우 분할
        splitter = QSplitter(Qt.Horizontal)

        # 왼쪽: 설정 + 제어
        left_widget = QWidget()
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(4, 4, 4, 4)

        self._initial_panel = InitialSettingsPanel(
            on_num_motors_changed=self._on_num_motors_changed
        )
        left_layout.addWidget(self._initial_panel)

        self._runtime_panel = RuntimeControlPanel()
        left_layout.addWidget(self._runtime_panel)

        left_widget.setLayout(left_layout)
        splitter.addWidget(left_widget)

        # 오른쪽: 모니터링
        self._monitoring_panel = MonitoringPanel()
        splitter.addWidget(self._monitoring_panel)

        # 비율 설정 (왼쪽 2 : 오른쪽 3)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)

        layout = QHBoxLayout()
        layout.addWidget(splitter)
        central.setLayout(layout)

        # 상태 표시줄
        self.statusBar().showMessage("Ready")

        # 초기 모터 위젯 구성
        self._on_num_motors_changed(self._initial_panel.get_num_motors())

    def _connect_signals(self):
        """시그널-슬롯 연결"""
        # Panel 1
        self._initial_panel.btn_start.clicked.connect(self._on_start_clicked)
        self._initial_panel.btn_stop.clicked.connect(self._on_stop_clicked)

        # Panel 2
        self._runtime_panel.btn_move_all.clicked.connect(self._on_move_all)
        self._runtime_panel.chk_coupling.toggled.connect(self._on_coupling_toggled)
        self._runtime_panel.slider_gain.valueChanged.connect(self._on_coupling_gain_changed)
        self._runtime_panel.btn_reset_sync.clicked.connect(self._on_reset_sync_error)

    def _update_panel_states(self):
        """앱 상태에 따라 패널 활성화/비활성화"""
        if self._app_state == "idle":
            self._initial_panel.setEnabled(True)
            self._initial_panel.btn_start.setEnabled(True)
            self._initial_panel.btn_stop.setEnabled(False)
            self._runtime_panel.setEnabled(False)
        elif self._app_state == "starting":
            self._initial_panel.setEnabled(False)
            self._runtime_panel.setEnabled(False)
        elif self._app_state == "running":
            self._initial_panel.setEnabled(False)
            self._initial_panel.btn_stop.setEnabled(True)
            self._runtime_panel.setEnabled(True)

    # ---- Panel 1 슬롯 ----

    def _on_num_motors_changed(self, n: int):
        """모터 수 변경 시 런타임 위젯 및 모니터링 재구성"""
        self._runtime_panel.rebuild_motor_widgets(
            n, on_origin=self._on_set_origin, on_move=self._on_move_motor
        )
        self._monitoring_panel.setup_for_motors(n)

    def _on_start_clicked(self):
        """EtherCAT 버스 시작"""
        adapter = self._initial_panel.get_adapter()
        num_motors = self._initial_panel.get_num_motors()
        cycle_time = self._initial_panel.get_cycle_time()
        motor_settings = self._initial_panel.get_motor_settings()
        max_sync_err = self._runtime_panel.get_max_sync_error()
        coupling_gain = self._runtime_panel.get_coupling_gain()
        coupling_enabled = self._runtime_panel.get_coupling_enabled()

        try:
            self._bus = EtherCATBusCoupling(
                adapter_name=adapter,
                num_slaves=num_motors,
                cycle_time_ms=cycle_time,
                max_sync_error_mm=max_sync_err,
                coupling_gain=coupling_gain,
                enable_coupling=coupling_enabled
            )

            # 모터별 설정 (start 전)
            for i, settings in enumerate(motor_settings):
                motor = self._bus.motors[i]
                motor.set_axis(settings['axis'])
                motor.set_profile_velocity(settings['velocity'])
                motor.set_profile_accel_decel(settings['accel'], settings['decel'])

            self._bus.start()

            # 그래프 데이터 초기화
            self._time_data.clear()
            self._position_data = {i: deque(maxlen=self._plot_max_points) for i in range(num_motors)}
            self._diff_data.clear()
            self._plot_start_time = None

            # 그래프 커브 초기화
            self._setup_plots_for_motors(num_motors, max_sync_err)

            # OP 상태 대기
            self._app_state = "starting"
            self._update_panel_states()
            self.statusBar().showMessage("Starting... waiting for OP state")

            self._startup_check_count = 0
            self._startup_timer = QTimer()
            self._startup_timer.timeout.connect(self._check_startup_ready)
            self._startup_timer.start(200)

        except Exception as e:
            QMessageBox.critical(self, "Start Error", f"Failed to create EtherCAT bus:\n{e}")
            if self._bus:
                self._bus.stop()
                self._bus = None
            self._app_state = "idle"
            self._update_panel_states()

    def _check_startup_ready(self):
        """OP 상태 도달 여부 폴링"""
        self._startup_check_count += 1

        if self._bus is None:
            self._startup_timer.stop()
            return

        try:
            all_ready = all(
                (m.status_word & 0x006F) == 0x0027
                for m in self._bus.motors
            )
        except Exception:
            all_ready = False

        if all_ready:
            self._startup_timer.stop()
            self._app_state = "running"
            self._update_panel_states()
            self._monitor_timer.start(100)
            self.statusBar().showMessage("Running - All motors operational")
        elif self._startup_check_count >= 50:  # 10초 타임아웃
            self._startup_timer.stop()
            QMessageBox.critical(self, "Startup Failed",
                "Motors did not reach Operation Enabled state within 10 seconds.\n"
                "Check adapter, cable, and motor power.")
            self._bus.stop()
            self._bus = None
            self._app_state = "idle"
            self._update_panel_states()
            self.statusBar().showMessage("Startup failed")

    def _on_stop_clicked(self):
        """EtherCAT 버스 정지"""
        self._monitor_timer.stop()
        if self._startup_timer:
            self._startup_timer.stop()

        if self._bus:
            self._bus.stop()
            self._bus = None

        self._app_state = "idle"
        self._update_panel_states()
        self.statusBar().showMessage("Stopped")

    # ---- Panel 2 슬롯 ----

    def _on_set_origin(self, motor_index: int):
        """원점 설정"""
        if self._bus:
            self._bus.motors[motor_index].set_origin()
            self.statusBar().showMessage(f"Motor {motor_index}: Origin set")

    def _on_move_motor(self, motor_index: int, target_mm: float):
        """개별 모터 이동"""
        if self._bus:
            self._bus.motors[motor_index].move_to_position_mm(target_mm)
            self.statusBar().showMessage(f"Motor {motor_index}: Moving to {target_mm:.2f} mm")

    def _on_move_all(self):
        """모든 모터 동시 이동"""
        if self._bus:
            target = self._runtime_panel.get_move_all_target()
            for motor in self._bus.motors:
                motor.move_to_position_mm(target)
            self.statusBar().showMessage(f"All motors: Moving to {target:.2f} mm")

    def _on_coupling_toggled(self, enabled: bool):
        """Cross Coupling 활성화/비활성화"""
        if self._bus:
            self._bus.coupling_enabled = enabled
            status = "enabled" if enabled else "disabled"
            self.statusBar().showMessage(f"Cross Coupling {status}")

    def _on_coupling_gain_changed(self, slider_value: int):
        """Cross Coupling 게인 변경"""
        gain = slider_value / 100.0
        self._runtime_panel.lbl_gain_value.setText(f"{gain:.2f}")
        if self._bus:
            self._bus.coupling_gain = gain

    def _on_reset_sync_error(self):
        """동기화 오류 리셋"""
        if self._bus:
            self._bus.reset_sync_error()
            self.statusBar().showMessage("Sync error reset")

    # ---- 모니터링 ----

    def _setup_plots_for_motors(self, n: int, max_sync_err: float):
        """모터 수에 맞게 그래프 커브 초기화"""
        # 위치 그래프 초기화
        self._monitoring_panel.plot_positions.clear()
        self._monitoring_panel.plot_positions.addLegend(offset=(10, 10))
        self._position_curves = {}
        for i in range(n):
            pen = pg.mkPen(color=MOTOR_COLORS[i % len(MOTOR_COLORS)], width=2)
            curve = self._monitoring_panel.plot_positions.plot(
                [], [], pen=pen, name=f"Motor {i}"
            )
            self._position_curves[i] = curve

        # 차이 그래프 초기화
        self._monitoring_panel.plot_diff.clear()
        self._diff_curve = self._monitoring_panel.plot_diff.plot(
            [], [], pen=pg.mkPen(color='#FF5722', width=2)
        )
        # 임계값 수평선
        self._threshold_line = pg.InfiniteLine(
            pos=max_sync_err, angle=0,
            pen=pg.mkPen(color='#F44336', width=1, style=Qt.DashLine),
            label=f"Threshold: {max_sync_err}mm",
            labelOpts={'color': '#F44336', 'position': 0.9}
        )
        self._monitoring_panel.plot_diff.addItem(self._threshold_line)

    def _update_monitoring(self):
        """100ms 주기 모니터링 업데이트"""
        if self._bus is None:
            return

        # 프로세스 생존 확인
        if not self._bus._process.is_alive():
            self._monitor_timer.stop()
            QMessageBox.warning(self, "Process Terminated",
                "EtherCAT bus process has terminated unexpectedly.")
            self._bus = None
            self._app_state = "idle"
            self._update_panel_states()
            self.statusBar().showMessage("Process terminated")
            return

        now = time.time()
        if self._plot_start_time is None:
            self._plot_start_time = now
        elapsed = now - self._plot_start_time
        self._time_data.append(elapsed)

        num_motors = len(self._bus.motors)

        # 테이블 업데이트 (깜빡임 방지)
        self._monitoring_panel.table.setUpdatesEnabled(False)

        for i in range(num_motors):
            motor = self._bus.motors[i]
            status_word = motor.status_word
            state_name = decode_cia402_state(status_word)
            pos_mm = motor.current_position_mm
            pos_pulse = motor.current_position_pulse
            moving = motor.is_moving()

            self._monitoring_panel.update_row(i, status_word, state_name, pos_mm, pos_pulse, moving)
            self._position_data[i].append(pos_mm)

        self._monitoring_panel.table.setUpdatesEnabled(True)

        # 위치 차이 계산
        if num_motors >= 2:
            diff = abs(self._bus.motors[0].current_position_mm -
                       self._bus.motors[1].current_position_mm)
            self._diff_data.append(diff)
            self._monitoring_panel.lbl_pos_diff.setText(f"M0-M1: {diff:.3f} mm")

        # Sync Error 인디케이터
        has_error = self._bus.has_sync_error
        self._monitoring_panel.update_sync_indicator(has_error)

        # 그래프 업데이트
        time_array = list(self._time_data)
        for i, curve in self._position_curves.items():
            if i in self._position_data:
                curve.setData(time_array, list(self._position_data[i]))

        if self._diff_curve is not None and num_motors >= 2:
            self._diff_curve.setData(time_array, list(self._diff_data))

        # 롤링 윈도우 (30초)
        if elapsed > 30:
            self._monitoring_panel.plot_positions.setXRange(elapsed - 30, elapsed)
            self._monitoring_panel.plot_diff.setXRange(elapsed - 30, elapsed)

    # ---- 종료 처리 ----

    def closeEvent(self, event):
        """창 닫기 시 안전 종료"""
        if self._bus is not None:
            reply = QMessageBox.question(
                self, "Confirm Exit",
                "EtherCAT bus is still running. Stop and exit?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self._on_stop_clicked()
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()


# ============================================================
# 진입점
# ============================================================
def main():
    import multiprocessing
    multiprocessing.freeze_support()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = MotorControlGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
