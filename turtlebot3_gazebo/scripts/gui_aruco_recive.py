from std_msgs.msg import Int32MultiArray
import rospy
import threading

class ArUcoIdListener:
    """监听多个机器人发布的 ArUco ID 并合并推送"""
    def __init__(self, on_ids_received):
        self.on_ids_received = on_ids_received  # 回调传 ID 列表
        self.collected_ids = set()
        self._start_thread()

    def _start_thread(self):
        thread = threading.Thread(target=self._ros_spin)
        thread.daemon = True
        thread.start()

    def _ros_spin(self):
        if not rospy.core.is_initialized():
            rospy.init_node("aruco_gui_listener", anonymous=True)

        rospy.Subscriber("/tb1/detected_ids", Int32MultiArray, self._callback)
        rospy.Subscriber("/tb2/detected_ids", Int32MultiArray, self._callback)

        rospy.spin()

    def _callback(self, msg):
        new_ids = set(msg.data)
        updated = new_ids - self.collected_ids  # 新增才更新 UI
        if updated:
            self.collected_ids.update(updated)
            sorted_ids = sorted(self.collected_ids)
            self.on_ids_received([str(i) for i in sorted_ids])
