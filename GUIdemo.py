import sys
import numpy as np
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas 
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QMessageBox,
    QTextEdit, QPushButton
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
        
        if self.log_widget:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_widget.append(f"[{timestamp}] {self.title} updated")

    def generate_data(self):
        """To be implemented in subclasses"""
        return np.random.rand(64, 64)
        

class CameraView(BaseDynamicImage):
    """Camera display view"""
    def __init__(self, camera_id=1, log_widget=None):
        title = f"Camera {camera_id}"
        super().__init__(title, log_widget, update_interval=1000)
        self.ax.set_axis_off()

class DynamicMap(BaseDynamicImage):
    """Map display view"""
    def __init__(self):
        super().__init__("Live Map", update_interval=1000)
        self.ax.set_axis_off()

    def generate_data(self):
        """Generate simulated map data"""
        x, y = np.meshgrid(
            np.linspace(0, 1, 64),
            np.linspace(0, 1, 64)
        )
        return np.sin(10*x) * np.cos(10*y) + np.random.normal(0, 0.1, (64, 64))

class SelectionWindow(QWidget):
    def __init__(self, log_widget=None):
        super().__init__()
        self.log_widget = log_widget
        self.initUI()
    
    def initUI(self):
        # Window Setting
        #self.setWindowTitle('选择框示例')
        #self.setGeometry(300, 300, 300, 150)
        
        # Create Layout
        layout = QVBoxLayout()
        
        # Creating a drop-down selection box
        self.combo_box = QComboBox(self)
        self.combo_box.addItems(['Apple', 'Banana', 'Orange', 'Grape'])  # Add options
        layout.addWidget(self.combo_box)
        
        # Create a confirmation button
        self.confirm_btn = QPushButton('Confirm', self)
        self.confirm_btn.clicked.connect(self.handle_selection)  # Connect click event
        layout.addWidget(self.confirm_btn)
        
        # Set layout
        self.setLayout(layout)
    
    def handle_selection(self):
        # Handle the selection confirmation event
        selected_item = self.combo_box.currentText()  # Get the currently selected text
        selected_index = self.combo_box.currentIndex()  # Get the currently selected index
        if self.log_widget:
            
            self.log_widget.append(f'Selected：{selected_item}\nIndex：{selected_index}')



class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.setWindowTitle("Monitoring System")
        self.resize(1600, 900)

    def initUI(self):
        main_layout = QVBoxLayout()

        # Map section
        self.map_view = DynamicMap()
        main_layout.addWidget(self.map_view, stretch=5)

        # Camera section
        cam_layout = QHBoxLayout()
        
        self.log1 = QTextEdit()
        self.log1.setPlaceholderText("Camera 1 Logs")
        self.log1.setMaximumHeight(100)
        
        self.log2 = QTextEdit()
        self.log2.setPlaceholderText("Camera 2 Logs")
        self.log2.setMaximumHeight(100)
        
        self.cam1 = CameraView(1, self.log1)
        self.sel1 = SelectionWindow(self.log1)
        self.cam2 = CameraView(2, self.log2)
        self.sel2 = SelectionWindow(self.log2)
        cam_layout.addWidget(self.cam1, stretch=1)
        cam_layout.addWidget(self.sel1, stretch=1)
        cam_layout.addWidget(self.cam2, stretch=1)
        cam_layout.addWidget(self.sel2, stretch=1)
        main_layout.addLayout(cam_layout, stretch=2)

        # Log section
        log_layout = QHBoxLayout()
        log_layout.addWidget(self.log1)
        log_layout.addWidget(self.log2)
        main_layout.addLayout(log_layout, stretch=1)

        # Control section
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


        main_layout.addLayout(control_layout)

        self.setLayout(main_layout)



    def Pause(self):
        """Pause all updates"""
        for w in [self.map_view, self.cam1, self.cam2]:
            w.timer.stop()
        
        # Show status in log
        if self.log1:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log1.append(f"[{timestamp}] System has been suspended")
            self.log2.append(f"[{timestamp}] System has been suspended")

    def Start(self):
        """Start all updates"""
        for w in [self.map_view, self.cam1, self.cam2]:
            w.timer.start()
        
        # Show status in log
        if self.log1:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log1.append(f"[{timestamp}] System has started")
            self.log2.append(f"[{timestamp}] System has started")


    def toggle_updates(self):
        """Toggle all updates"""
        widgets = [self.map_view, self.cam1, self.cam2]
        
        if any(w.timer.isActive() for w in widgets):
            for w in widgets:
                w.timer.stop()
            self.btn_toggle_pause.setText("Resume Updates")
        else:
            for w in widgets:
                w.timer.start()
            self.btn_toggle_pause.setText("Pause Updates")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())