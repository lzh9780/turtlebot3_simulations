#!/usr/bin/env python
import rospy
import tf
import math
import tkinter as tk

class PoseGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("TurtleBot3 实时位置显示")
        self.label_x = tk.Label(root, text="X: --", font=('Arial', 16))
        self.label_y = tk.Label(root, text="Y: --", font=('Arial', 16))
        self.label_yaw = tk.Label(root, text="Yaw: --", font=('Arial', 16))
        self.label_x.pack(pady=5)
        self.label_y.pack(pady=5)
        self.label_yaw.pack(pady=5)

        self.listener = tf.TransformListener()

        self.update_pose()

    def update_pose(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            x = trans[0]
            y = trans[1]
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            yaw_deg = math.degrees(yaw)

            self.label_x.config(text=f"X: {x:.2f} m")
            self.label_y.config(text=f"Y: {y:.2f} m")
            self.label_yaw.config(text=f"Yaw: {yaw_deg:.2f}°")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        self.root.after(100, self.update_pose)  # 每100毫秒更新一次

if __name__ == '__main__':
    rospy.init_node('robot_pose_gui_node')
    root = tk.Tk()
    app = PoseGUI(root)
    root.mainloop()

# 可以用 但是必须同时运行的有turtlenav.launch