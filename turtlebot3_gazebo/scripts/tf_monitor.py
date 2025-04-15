#!/usr/bin/env python
import rospy
import tf
import math

def get_pose():
    rospy.init_node('tf_pose_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10)  # 10Hz 刷新

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            x = trans[0]
            y = trans[1]
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            yaw_deg = math.degrees(yaw)
            print("x: {:.2f}, y: {:.2f}, yaw: {:.2f}°".format(x, y, yaw_deg))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()

if __name__ == '__main__':
    get_pose()
# 必须运行turtlenav.launch才有数据