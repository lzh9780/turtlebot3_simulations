#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def publish_pose(tf_listener, source_frame, pub, robot_name):
    try:
        now = rospy.Time(0)
        tf_listener.waitForTransform("/map", source_frame, now, rospy.Duration(1.0))
        (trans, rot) = tf_listener.lookupTransform("/map", source_frame, now)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = trans[0]
        pose_msg.pose.position.y = trans[1]
        pose_msg.pose.position.z = trans[2]
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]

        pub.publish(pose_msg)
    except Exception as e:
        rospy.logwarn("[%s] Transform error: %s", robot_name, e)

def main():
    rospy.init_node("pose_broadcaster")
    tf_listener = tf.TransformListener()

    pub1 = rospy.Publisher("/tb1/pose_info", PoseStamped, queue_size=10)
    pub2 = rospy.Publisher("/tb2/pose_info", PoseStamped, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        publish_pose(tf_listener, "turtlebot3_1/base_footprint", pub1, "tb1")
        publish_pose(tf_listener, "turtlebot3_2/base_footprint", pub2, "tb2")
        rate.sleep()

if __name__ == '__main__':
    main()
