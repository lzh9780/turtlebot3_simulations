import rospy
import threading
import numpy as np
import cv2
import json
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, QObject
import tf.transformations


class RosMapWithRobotAndPathWidget(QLabel):
    def __init__(self, map_topic="/map", robot_topics=None, path_topics=None, parent=None):
        super().__init__(parent)
        self.setText("Waiting for map and path...")
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: black; color: white;")
        self.setFixedSize(400, 400)

        self.map_topic = map_topic
        self.robot_topics = robot_topics or {}
        self.path_topics = path_topics or {}

        self.map_data = None
        self.map_info = None
        self.robot_poses = {}
        self.paths = {}  # 结构：{name: [(x, y), ...]}

        self.updater = _RosMapSignal()
        self.updater.map_signal.connect(self.update_display)
        
        self.block_positions = []

        self._start_ros_thread()

    def _start_ros_thread(self):
        thread = threading.Thread(target=self._ros_thread)
        thread.daemon = True
        thread.start()

    def _ros_thread(self):
        if not rospy.core.is_initialized():
            rospy.init_node('ros_map_multi_path_gui_node', anonymous=True)

        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_callback)

        for name, topic in self.robot_topics.items():
            rospy.Subscriber(topic, PoseWithCovarianceStamped, lambda msg, n=name: self._pose_callback(n, msg))

        for name, topic in self.path_topics.items():
            rospy.Subscriber(topic, Path, lambda msg, n=name: self._path_callback(n, msg))
            
        rospy.Subscriber("/tb3/info", String, self._block_callback)

        rospy.spin()

    def _map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.map_info = msg.info

        data = np.flipud(np.array(msg.data, dtype=np.int8).reshape((height, width)))
        gray_map = np.zeros((height, width), dtype=np.uint8)
        gray_map[data == 0] = 255
        gray_map[data == 100] = 0
        gray_map[data == -1] = 128

        self.map_data = gray_map
        self.updater.map_signal.emit("update")

    def _pose_callback(self, name, msg):
        self.robot_poses[name] = msg.pose.pose
        self.updater.map_signal.emit("update")

    def _path_callback(self, name, msg):
        self.paths[name] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.updater.map_signal.emit("update")
    
    def _block_callback(self, msg:String):
        dic = json.loads(msg.data)
        try:
            try:
                if dic["info"]["pose"] != None:
                    x, y, z, rx, ry, rz, rw = dic["info"]["pose"]
            except (ValueError, TypeError):
                print("Invaild pose: ", dic["info"]["pose"])
                print("message: ", dic)
            
            if dic['type'] != "robot":
                for b in self.block_positions:
                    if b[0] == f"{dic['type']} {dic['id']}":
                        b[1] = dic["info"]["pose"][0]
                        b[2] = dic["info"]["pose"][1]
                        return
                self.block_positions.append((f"{dic['type']} {dic['id']}", dic["info"]["pose"][0], dic["info"]["pose"][1]))
        except:
            pass
        
        self.updater.map_signal.emit("update")

    def update_display(self, _):
        if self.map_data is None or self.map_info is None:
            return

        img = cv2.cvtColor(self.map_data.copy(), cv2.COLOR_GRAY2BGR)
        res = self.map_info.resolution
        origin = self.map_info.origin.position

        def world_to_pixel(x, y):
            px = int((x - origin.x) / res)
            py = img.shape[0] - int((y - origin.y) / res)
            return px, py


        # 多条路径：不同颜色
        path_colors = [(255, 0, 0), (0, 255, 255), (255, 255, 0), (0, 0, 255)]  # 支持 4 个机器人
        for idx, (name, path) in enumerate(self.paths.items()):
            color = path_colors[idx % len(path_colors)]
            for i in range(1, len(path)):
                x1, y1 = world_to_pixel(*path[i - 1])
                x2, y2 = world_to_pixel(*path[i])
                cv2.line(img, (x1, y1), (x2, y2), color, 2)
        
        for blocks in self.block_positions:
            block_id, x, y = blocks
            px, py = world_to_pixel(x, y)
            cv2.circle(img, (px, py), 6, (0, 255, 255), -1) # 黄点
            cv2.putText(img, block_id, (px + 5, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        # 画机器人位置
        for name, pose in self.robot_poses.items():
            x, y = world_to_pixel(pose.position.x, pose.position.y)
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(img, name, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

            # 朝向箭头
            q = pose.orientation
            _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            arrow_length = 15
            x2 = int(x + arrow_length * np.cos(-yaw))
            y2 = int(y + arrow_length * np.sin(-yaw))
            cv2.arrowedLine(img, (x, y), (x2, y2), (255, 0, 0), 2, tipLength=0.3)

        h, w, _ = img.shape
        q_img = QImage(img.data, w, h, 3 * w, QImage.Format_RGB888)
        self.setPixmap(QPixmap.fromImage(q_img).scaled(self.width(), self.height(), Qt.KeepAspectRatio))


class _RosMapSignal(QObject):
    map_signal = pyqtSignal(object)
