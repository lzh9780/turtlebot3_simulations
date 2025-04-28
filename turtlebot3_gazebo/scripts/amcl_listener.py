#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback_amcl_pose(msg, robot_name):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    print(f"[{robot_name}] Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}")
    print(f"[{robot_name}] Orientation (quaternion): x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f}")
    print("---------------------------------------------------")

def listener():
    rospy.init_node('amcl_pose_listener', anonymous=True)

    # 订阅两个机器人的 AMCL 位姿话题
    rospy.Subscriber("/tb3_1/amcl_pose", PoseWithCovarianceStamped, callback_amcl_pose, callback_args="tb3_1")
    rospy.Subscriber("/tb3_2/amcl_pose", PoseWithCovarianceStamped, callback_amcl_pose, callback_args="tb3_2")

    rospy.spin()

if __name__ == '__main__':
    listener()
