import rospy
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, QObject


class RosCvCameraWidget(QLabel):
    """显示 ROS 图像并进行 ArUco 检测，嵌入到 PyQt5 界面中"""
    
    def __init__(self, topic_name, label="Camera View", parent=None, show_aruco=True):
        super().__init__(parent)
        self.topic_name = topic_name
        self.label = label
        self.show_aruco = show_aruco

        self.setText(f"Waiting for: {label}")
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: black; color: white;")
        self.setFixedSize(320, 240)

        self.bridge = CvBridge()
        self.updater = _RosCameraSignal()
        self.updater.image_signal.connect(self.update_image)

        # ArUco 初始化
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)
        # self.aruco_params = cv2.aruco.DetectorParameters_create()

        self._start_ros_thread()

    def _start_ros_thread(self):
        thread = threading.Thread(target=self._ros_thread)
        thread.daemon = True
        thread.start()

    def _ros_thread(self):
        if not rospy.core.is_initialized():
            rospy.init_node('ros_cv_camera_gui_node', anonymous=True)
        rospy.Subscriber(self.topic_name, Image, self._callback)
        rospy.spin()

    def _callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.updater.image_signal.emit(cv_img)
        except Exception as e:
            print(f"[ROS Camera ERROR] {e}")

    def update_image(self, cv_img):
        """接收图像并可选地绘制 ArUco 检测结果"""
        display_img = cv_img.copy()

        if self.show_aruco:
            corners, ids, _ = self.detector.detectMarkers(display_img)
            if ids is not None:
                # 画出边框
                cv2.aruco.drawDetectedMarkers(display_img, corners, ids)
                for i, corner in enumerate(corners):
                    c = corner[0]
                    center_x = int(c[:, 0].mean())
                    center_y = int(c[:, 1].mean())
                    cv2.circle(display_img, (center_x, center_y), 4, (0, 255, 0), -1)
                    cv2.putText(display_img, f"ID: {ids[i][0]}", (center_x - 10, center_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        height, width, _ = display_img.shape
        qimg = QImage(display_img.data, width, height, 3 * width, QImage.Format_RGB888).rgbSwapped()
        self.setPixmap(QPixmap.fromImage(qimg).scaled(self.width(), self.height(), Qt.KeepAspectRatio))


class _RosCameraSignal(QObject):
    image_signal = pyqtSignal(object)



