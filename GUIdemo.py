import sys
import numpy as np
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas 
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QMessageBox,
    QTextEdit, QPushButton, QLabel
)
from datetime import datetime

class BaseDynamicImage(QWidget):
    """Base class for dynamic image displays"""
    def __init__(self, title="Dynamic View", log_widget=None, update_interval=1000):
        super().__init__()
        self.title = title
        self.log_widget = log_widget
        self.update_interval = update_interval
        self.init_ui()
        self.setup_timer()

    def init_ui(self):
        self.figure = Figure(figsize=(6, 4))
        self.canvas = FigureCanvas(self.figure)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        
        self.ax = self.figure.add_subplot(111)
        self.img = self.ax.imshow(
            np.random.rand(64, 64),
            cmap='viridis',
            interpolation='nearest'
        )
        self.ax.set_title(self.title)

    def setup_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(self.update_interval)

    def update_image(self):
        new_data = self.generate_data()
        self.img.set_data(new_data)
        self.canvas.draw_idle()
        '''
        if self.log_widget:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_widget.append(f"[{timestamp}] {self.title} updated")
        '''
    def generate_data(self):
        """To be implemented in subclasses"""
        return np.random.rand(64, 64)
        

class CameraView(BaseDynamicImage):
    """Camera display view"""
    def __init__(self, camera_id=1, log_widget=None):
        title = f"TurtleBot{camera_id} Camera"
        super().__init__(title, log_widget, update_interval=100)
        self.ax.set_axis_off()

class DynamicMap(BaseDynamicImage):
    """Map display view"""
    def __init__(self):
        super().__init__("Live Map", update_interval=100)
        self.ax.set_axis_off()

    def generate_data(self):
        """Generate simulated map data"""
        x, y = np.meshgrid(
            np.linspace(0, 1, 64),
            np.linspace(0, 1, 64)
        )
        return np.sin(10*x) * np.cos(10*y) + np.random.normal(0, 0.1, (64, 64))

class RobotControl(QWidget):
    def __init__(self, log_widget=None, camera_widget=None, main_control=None):
        super().__init__()
        self.log_widget = log_widget
        self.camera = camera_widget
        self.main_control = main_control
        self.options = []
        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.enable_done_button)
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        self.combo_box = QComboBox(self)
        layout.addWidget(self.combo_box)

        self.confirm_btn = QPushButton('Confirm', self)
        self.confirm_btn.clicked.connect(self.handle_selection)

        self.pause_btn = QPushButton('Pause Camera', self)
        self.pause_btn.clicked.connect(self.toggle_camera)

        self.done_btn = QPushButton('Finish', self)
        self.done_btn.setEnabled(False)
        self.done_btn.clicked.connect(self.finish_operation)

        layout.addWidget(self.confirm_btn)
        layout.addWidget(self.pause_btn)
        layout.addWidget(self.done_btn)
        self.setLayout(layout)

    def handle_selection(self):
        selected = self.combo_box.currentText()
        if not selected:
            if self.log_widget:
                self.log_widget.append("⚠️ No selected ID")
            return

        timestamp = datetime.now().strftime("%H:%M:%S")
        if self.log_widget:
            self.log_widget.append(f"[{timestamp}] Confirm ID selected：{selected}， finish after 10 second")

        # Start countdown 10 seconds
        self.timer.start(10000)
        self.done_btn.setEnabled(False)

    def enable_done_button(self):
        self.done_btn.setEnabled(True)
        if self.log_widget:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_widget.append(f"[{timestamp}] Finish button enable")


    def toggle_camera(self):
        if self.camera.timer.isActive():
            self.camera.timer.stop()
            self.pause_btn.setText("Resume Camera")
            status = "Pause"
        else:
            self.camera.timer.start()
            self.pause_btn.setText("Pause Camera")
            status = "Resume"

        if self.log_widget:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_widget.append(f"[{timestamp}] Camera {status}")
    def load_options(self, options):
        self.combo_box.clear()
        self.combo_box.addItems(options)
        self.done_btn.setEnabled(False)

    def finish_operation(self):
        selected_value = self.combo_box.currentText()
        if self.main_control:
            self.main_control.disable_combobox_by_value(selected_value)
        if self.log_widget:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_widget.append(f"[{timestamp}] Object {selected_value} Get, option {selected_value} disableed")

class MainControl(QWidget):
    def __init__(self, log_widget=None):
        super().__init__()
        self.log_widget = log_widget or QTextEdit()
        self.available_options = []
        self.data_loaded = False
        self.on_submit_callback = None  # 添加 callback
        self.initUI()
        self.setup_options_refresh()

    def initUI(self):
        main_layout = QVBoxLayout()

        # 刷新按钮
        self.refresh_btn = QPushButton("刷新选项", self)
        self.refresh_btn.clicked.connect(self.fetch_options)

        # 下拉菜单
        self.combobox1 = QComboBox(self)
        self.combobox1.setEnabled(False)
        self.combobox1.currentIndexChanged.connect(self.update_combobox_options)

        self.combobox2 = QComboBox(self)
        self.combobox2.setEnabled(False)
        self.combobox2.currentIndexChanged.connect(self.update_combobox_options)

        self.combobox3 = QComboBox(self)
        self.combobox3.setEnabled(False)
        self.combobox3.currentIndexChanged.connect(self.update_combobox_options)
        # 提交按钮  
        self.submit_btn = QPushButton("Submit", self)
        self.submit_btn.setEnabled(False)
        self.submit_btn.clicked.connect(self.handle_submit)

        # 状态显示
        self.status_label = QLabel("Waiting for data...")
        self.status_label.setStyleSheet("color: gray;")

        # 日志区域
        self.log = QTextEdit()
        self.log.setMaximumHeight(150)
        self.log.setReadOnly(True)

        # 添加控件到布局
        main_layout.addWidget(self.refresh_btn)
        main_layout.addWidget(QLabel("Marker ID:"))
        main_layout.addWidget(self.combobox1)
        main_layout.addWidget(QLabel("Marker ID:"))
        main_layout.addWidget(self.combobox2)
        main_layout.addWidget(QLabel("Marker ID:"))
        main_layout.addWidget(self.combobox3)
        main_layout.addWidget(self.status_label)
        main_layout.addWidget(self.submit_btn)
        main_layout.addWidget(QLabel("Logs:"))
        main_layout.addWidget(self.log)

        self.setLayout(main_layout)

    def setup_options_refresh(self):
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.fetch_options)
        self.refresh_timer.start(5000)

    def fetch_options(self):
        if self.data_loaded:
            return

        try:
            options = self.simulate_data_fetch()
            log_entry = f"[{datetime.now().strftime('%H:%M:%S')}] Get selected ID: {options}"
            self.log_widget.append(log_entry)

            if len(options) >= 6:
                self.available_options = options
                self.update_comboboxes()
                self.set_controls_enabled(True)
                self.status_label.setText("Data loaded")
                self.status_label.setStyleSheet("color: green;")
                self.refresh_timer.stop()
                self.data_loaded = True
                self.log.append("Get 6 Option")
            else:
                self.available_options = []
                self.clear_comboboxes()
                self.set_controls_enabled(False)
                self.status_label.setText(f"Not enough data：{len(options)}/3")
                self.status_label.setStyleSheet("color: red;")
                self.log.append("Verification failed, keep trying...")

        except Exception as e:
            self.log.append(f"[ERROR] Data acquisition failed: {str(e)}")
            self.set_controls_enabled(False)

    def simulate_data_fetch(self):
        if not self.data_loaded:
            num_options = np.random.choice([0, 6])
            sample_options = ['A', 'B', 'C', 'D', 'E', 'F']
            return sample_options[:num_options]
        return []

    def update_comboboxes(self):
        for cb in [self.combobox1, self.combobox2, self.combobox3]:
            cb.clear()
            cb.addItems(self.available_options)
            cb.setCurrentIndex(-1)

    def clear_comboboxes(self):
        for cb in [self.combobox1, self.combobox2, self.combobox3]:
            cb.clear()

    def set_controls_enabled(self, enabled):
        for cb in [self.combobox1, self.combobox2, self.combobox3]:
            cb.setEnabled(enabled)
        self.submit_btn.setEnabled(enabled)


    def update_combobox_options(self):
        # Get the currently selected value
        selections = {
            'combobox1': self.combobox1.currentText(),
            'combobox2': self.combobox2.currentText(),
            'combobox3': self.combobox3.currentText(),
        }

        # Prepare for the remaining options available
        used_values = {v for v in selections.values() if v}
        for name, box in [('combobox1', self.combobox1), 
                        ('combobox2', self.combobox2), 
                        ('combobox3', self.combobox3)]:
            current = selections[name]
            box.blockSignals(True)  # Prevent recursive triggering
            
            box.clear()
            for opt in self.available_options:
                if opt == current or opt not in used_values:
                    box.addItem(opt)

            if current in self.available_options:
                box.setCurrentText(current)
            else:
                box.setCurrentIndex(-1)

            box.blockSignals(False)


    def set_submit_callback(self, callback):
        self.on_submit_callback = callback

    def get_selected_mapping(self):
        return {
            self.combobox1.currentText(): self.combobox1,
            self.combobox2.currentText(): self.combobox2,
            self.combobox3.currentText(): self.combobox3,
        }

    def disable_combobox_by_value(self, value):
        mapping = self.get_selected_mapping()
        if value in mapping:
            mapping[value].setEnabled(False)
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log.append(f"[{timestamp}] Disabled option：{value}")

    def handle_submit(self):
        selected1 = self.combobox1.currentText()
        selected2 = self.combobox2.currentText()
        selected3 = self.combobox3.currentText()
        timestamp = datetime.now().strftime("%H:%M:%S")


        selections = [selected1, selected2, selected3]
        if len(set(selections)) < 3:
            self.log.append(f"[{timestamp}] Submission failed")
            self.status_label.setText("Submit failed: Duplicate selection")
            self.status_label.setStyleSheet("color: red;")
            return

        self.log_widget.append(
            f"[{timestamp}] Submit: [{selected1}, {selected2}, {selected3} to Turtlebot]\n"
            f"Full list of options: {self.available_options}"
        )

        self.log.append(f"[{timestamp}]  Submission successful: {selected1}, {selected2}, {selected3}")
        self.status_label.setText("Submission successful")
        self.status_label.setStyleSheet("color: blue;")

        # Call the callback function push to RobotControl
        if self.on_submit_callback:
            self.on_submit_callback(selections)
       
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.setWindowTitle("Monitoring System")
        self.resize(1600, 900)
    
    def initUI(self):
        main_v_layout = QVBoxLayout()

        map_control_layout = QHBoxLayout()
        self.map_view = DynamicMap()
        map_control_layout.addWidget(self.map_view, stretch=4)

        self.main_control = MainControl()
        map_control_layout.addWidget(self.main_control, stretch=2)

        main_v_layout.addLayout(map_control_layout, stretch=5)

        cam_layout = QHBoxLayout()
        self.log1 = QTextEdit()
        self.log1.setPlaceholderText("Turtlebot 1 Logs")
        self.log1.setMaximumHeight(100)
        self.log1.setReadOnly(True)

        self.log2 = QTextEdit()
        self.log2.setPlaceholderText("Turtlebot 2 Logs")
        self.log2.setMaximumHeight(100)
        self.log2.setReadOnly(True)
        self.cam1 = CameraView(1, self.log1)
        self.cam2 = CameraView(2, self.log2)

        self.sel1 = RobotControl(self.log1, self.cam1, self.main_control)
        self.sel2 = RobotControl(self.log2, self.cam2, self.main_control)

        cam_layout.addWidget(self.cam1, stretch=1)
        cam_layout.addWidget(self.sel1, stretch=1)
        cam_layout.addWidget(self.cam2, stretch=1)
        cam_layout.addWidget(self.sel2, stretch=1)
        main_v_layout.addLayout(cam_layout, stretch=2)

        log_layout = QHBoxLayout()
        log_layout.addWidget(self.log1)
        log_layout.addWidget(self.log2)
        main_v_layout.addLayout(log_layout, stretch=1)

        control_layout = QHBoxLayout()
        self.btn_toggle_start = QPushButton("Start")
        self.btn_toggle_start.clicked.connect(self.Start)

        self.btn_toggle_pause = QPushButton("Pause")
        self.btn_toggle_pause.clicked.connect(self.Pause)

        self.btn_toggle_reset = QPushButton("Reset")

        control_layout.addStretch()
        control_layout.addWidget(self.btn_toggle_start)
        control_layout.addWidget(self.btn_toggle_pause)
        control_layout.addWidget(self.btn_toggle_reset)
        control_layout.addStretch()

        main_v_layout.addLayout(control_layout)
        self.setLayout(main_v_layout)

        # Bind push callback
        self.main_control.set_submit_callback(self.push_to_robot)

    def push_to_robot(self, options):
        self.sel1.load_options(options)
        self.sel2.load_options(options)

    def Pause(self):
        """Pause all updates"""
        for w in [self.map_view]:
            w.timer.stop()
        
        # Show status in log
        if self.log1:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log1.append(f"[{timestamp}] System has been suspended")
            self.log2.append(f"[{timestamp}] System has been suspended")

    def Start(self):
        """Start all updates"""
        for w in [self.map_view]:
            w.timer.start()
        
        # Show status in log
        if self.log1:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log1.append(f"[{timestamp}] System has started")
            self.log2.append(f"[{timestamp}] System has started")



if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
