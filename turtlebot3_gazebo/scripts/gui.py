import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QComboBox,
    QTextEdit, QPushButton, QLabel
)
from PyQt5.QtCore import QTimer
from datetime import datetime
from gui_camera import RosCvCameraWidget
from gui_map import RosMapWithRobotAndPathWidget
from gui_aruco_recive import ArUcoIdListener


class RobotControl(QWidget):
    def __init__(self, log_widget, camera_widget, main_control):
        super().__init__()
        self.log_widget = log_widget
        self.camera = camera_widget
        self.main_control = main_control
        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.enable_done_button)
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        self.combo_box = QComboBox()
        self.confirm_btn = QPushButton('Confirm')
        self.confirm_btn.clicked.connect(self.handle_selection)

        self.pause_btn = QPushButton('Pause Camera')
        self.pause_btn.clicked.connect(self.toggle_camera)

        self.done_btn = QPushButton('Finish')
        self.done_btn.setEnabled(False)
        self.done_btn.clicked.connect(self.finish_operation)

        layout.addWidget(self.combo_box)
        layout.addWidget(self.confirm_btn)
        layout.addWidget(self.pause_btn)
        layout.addWidget(self.done_btn)
        self.setLayout(layout)

    def handle_selection(self):
        selected = self.combo_box.currentText()
        if not selected:
            self.log_widget.append("No selected ID")
            return

        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_widget.append(f"[{timestamp}] Confirm ID: {selected}, finish after 10 sec")
        self.timer.start(10000)
        self.done_btn.setEnabled(False)

    def enable_done_button(self):
        self.done_btn.setEnabled(True)
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_widget.append(f"[{timestamp}] Finish button enabled")

    def toggle_camera(self):
        if self.camera.timer.isActive():
            self.camera.timer.stop()
            self.pause_btn.setText("Resume Camera")
            status = "Paused"
        else:
            self.camera.timer.start()
            self.pause_btn.setText("Pause Camera")
            status = "Resumed"

        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_widget.append(f"[{timestamp}] Camera {status}")

    def load_options(self, options):
        self.combo_box.clear()
        self.combo_box.addItems(options)
        self.done_btn.setEnabled(False)

    def finish_operation(self):
        selected_value = self.combo_box.currentText()
        self.main_control.disable_combobox_by_value(selected_value)
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_widget.append(f"[{timestamp}] Object {selected_value} acquired and disabled")


class MainControl(QWidget):
    def __init__(self, log_widget):
        super().__init__()
        self.log_widget = log_widget
        self.available_options = []
        self.on_submit_callback = None
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        self.combobox1 = QComboBox(); self.combobox1.setEnabled(False)
        self.combobox2 = QComboBox(); self.combobox2.setEnabled(False)
        self.combobox3 = QComboBox(); self.combobox3.setEnabled(False)

        self.submit_btn = QPushButton("Submit")
        self.submit_btn.setEnabled(False)
        self.submit_btn.clicked.connect(self.handle_submit)

        self.status_label = QLabel("Waiting for data...")
        self.status_label.setStyleSheet("color: gray;")

        for cb in [self.combobox1, self.combobox2, self.combobox3]:
            cb.currentIndexChanged.connect(self.update_combobox_options)
            layout.addWidget(QLabel("Marker ID:"))
            layout.addWidget(cb)

        layout.addWidget(self.status_label)
        layout.addWidget(self.submit_btn)
        self.setLayout(layout)

    def update_comboboxes(self):
        for cb in [self.combobox1, self.combobox2, self.combobox3]:
            cb.clear(); cb.addItems(self.available_options)
            cb.setCurrentIndex(-1)

    def set_controls_enabled(self, enabled):
        for cb in [self.combobox1, self.combobox2, self.combobox3]:
            cb.setEnabled(enabled)
        self.submit_btn.setEnabled(enabled)

    def update_combobox_options(self):
        selections = {cb.currentText() for cb in [self.combobox1, self.combobox2, self.combobox3] if cb.currentText()}
        for cb in [self.combobox1, self.combobox2, self.combobox3]:
            current = cb.currentText()
            cb.blockSignals(True)
            cb.clear()
            for opt in self.available_options:
                if opt == current or opt not in selections:
                    cb.addItem(opt)
            cb.setCurrentText(current)
            cb.blockSignals(False)

    def disable_combobox_by_value(self, value):
        for cb in [self.combobox1, self.combobox2, self.combobox3]:
            if cb.currentText() == value:
                cb.setEnabled(False)
                timestamp = datetime.now().strftime("%H:%M:%S")
                self.log_widget.append(f"[{timestamp}] Disabled option: {value}")

    def handle_submit(self):
        selections = [self.combobox1.currentText(), self.combobox2.currentText(), self.combobox3.currentText()]
        timestamp = datetime.now().strftime("%H:%M:%S")

        if len(set(selections)) < 3:
            self.log_widget.append(f"[{timestamp}] Submission failed: Duplicate selection")
            self.status_label.setText("Submit failed: Duplicate selection")
            self.status_label.setStyleSheet("color: red;")
            return

        self.log_widget.append(f"[{timestamp}] Submit: {selections}")
        self.status_label.setText("Submission successful")
        self.status_label.setStyleSheet("color: blue;")

        if self.on_submit_callback:
            self.on_submit_callback(selections)

    def set_submit_callback(self, callback):
        self.on_submit_callback = callback


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Monitoring System")
        self.resize(1600, 900)
        self.initUI()
        self.aruco_listener = ArUcoIdListener(self.push_ids_to_maincontrol)

    def initUI(self):
        main_layout = QVBoxLayout()

        # Map + Control Panel
        map_layout = QHBoxLayout()
        self.map_view = RosMapWithRobotAndPathWidget(
            map_topic="/map",
            path_topic="/move_base/GlobalPlanner/plan",
            robot_topics={"TurtleBot1": "/tb1/amcl_pose", "TurtleBot2": "/tb2/amcl_pose"}
        )
        self.log1 = QTextEdit(); self.log1.setReadOnly(True)
        self.log2 = QTextEdit(); self.log2.setReadOnly(True)

        self.main_control = MainControl(self.log1)
        self.main_control.set_submit_callback(self.push_to_robot)

        map_layout.addWidget(self.map_view, stretch=4)
        map_layout.addWidget(self.main_control, stretch=2)
        main_layout.addLayout(map_layout)

        # Cameras
        cam_layout = QHBoxLayout()
        self.cam1 = RosCvCameraWidget("/camera1/image_raw", "TurtleBot1", show_aruco=True)
        self.cam2 = RosCvCameraWidget("/camera2/image_raw", "TurtleBot2", show_aruco=True)
        self.sel1 = RobotControl(self.log1, self.cam1, self.main_control)
        self.sel2 = RobotControl(self.log2, self.cam2, self.main_control)

        cam_layout.addWidget(self.cam1)
        cam_layout.addWidget(self.sel1)
        cam_layout.addWidget(self.cam2)
        cam_layout.addWidget(self.sel2)
        main_layout.addLayout(cam_layout)

        # Logs
        log_layout = QHBoxLayout()
        log_layout.addWidget(self.log1)
        log_layout.addWidget(self.log2)
        main_layout.addLayout(log_layout)

        self.setLayout(main_layout)

    def push_ids_to_maincontrol(self, ids):
        self.main_control.available_options = ids
        self.main_control.update_comboboxes()
        enable = len(ids) >= 3
        self.main_control.set_controls_enabled(enable)
        self.main_control.status_label.setText("Ready" if enable else "Waiting for more IDs...")
        self.main_control.status_label.setStyleSheet("color: green;" if enable else "color: gray;")

        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log1.append(f"[{timestamp}] Updated ArUco ID list: {ids}")

    def push_to_robot(self, options):
        self.sel1.load_options(options)
        self.sel2.load_options(options)
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log1.append(f"[{timestamp}] Options pushed to RobotControl")
        self.log2.append(f"[{timestamp}] Options pushed to RobotControl")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
