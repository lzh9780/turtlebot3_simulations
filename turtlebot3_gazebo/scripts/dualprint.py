#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
import math

def quaternion_to_yaw(orientation_q):
    """四元数转yaw角（弧度）"""
    (_, _, yaw) = tf.transformations.euler_from_quaternion(
        [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    )
    return yaw

class MultiRobotPosePrinter:
    def __init__(self):
        rospy.init_node('multi_robot_pose_printer', anonymous=True)

        self.tb3_1_pose = None
        self.tb3_2_pose = None

        # 订阅两个命名空间下的 amcl_pose
        rospy.Subscriber("/tb3_1/amcl_pose", PoseWithCovarianceStamped, self.tb3_1_callback)
        rospy.Subscriber("/tb3_2/amcl_pose", PoseWithCovarianceStamped, self.tb3_2_callback)

        rospy.Timer(rospy.Duration(0.1), self.print_poses)  # 每秒打印一次

    def tb3_1_callback(self, msg):
        self.tb3_1_pose = msg.pose.pose

    def tb3_2_callback(self, msg):
        self.tb3_2_pose = msg.pose.pose

    def print_poses(self, event):
        if self.tb3_1_pose:
            yaw1 = quaternion_to_yaw(self.tb3_1_pose.orientation)
            print("TB3_1: x = {:.2f}, y = {:.2f}, yaw = {:.2f} rad".format(
                self.tb3_1_pose.position.x,
                self.tb3_1_pose.position.y,
                yaw1
            ))
        else:
            print("TB3_1 pose not received yet.")

        if self.tb3_2_pose:
            yaw2 = quaternion_to_yaw(self.tb3_2_pose.orientation)
            print("TB3_2: x = {:.2f}, y = {:.2f}, yaw = {:.2f} rad".format(
                self.tb3_2_pose.position.x,
                self.tb3_2_pose.position.y,
                yaw2
            ))
        else:
            print("TB3_2 pose not received yet.")

if __name__ == '__main__':
    try:
        MultiRobotPosePrinter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
