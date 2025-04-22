#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

tb1_pose = None
tb2_pose = None

def tb1_odom_callback(msg):
    global tb1_pose
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    tb1_pose = (pos.x, pos.y, yaw)

def tb2_odom_callback(msg):
    global tb2_pose
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    tb2_pose = (pos.x, pos.y, yaw)

def main():
    rospy.init_node('dual_odom_printer')
    rospy.Subscriber('/tb1/odom', Odometry, tb1_odom_callback)
    rospy.Subscriber('/tb2/odom', Odometry, tb2_odom_callback)

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        if tb1_pose and tb2_pose:
            print("TB1 -> x: {:.2f}, y: {:.2f}, θ: {:.2f}".format(*tb1_pose))
            print("TB2 -> x: {:.2f}, y: {:.2f}, θ: {:.2f}".format(*tb2_pose))
            print("-" * 50)
        rate.sleep()

if __name__ == '__main__':
    main()
