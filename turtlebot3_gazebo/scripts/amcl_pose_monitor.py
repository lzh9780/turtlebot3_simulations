#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import math

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = math.degrees(euler[2])  # 转换为角度
    print("Robot Position => x: {:.2f}, y: {:.2f}, yaw: {:.2f}°".format(x, y, yaw))

def listener():
    rospy.init_node('amcl_pose_listener', anonymous=True)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


# disadvantage: 更新频率不够, 且受单位时间内移动距离和角度阈值的影响