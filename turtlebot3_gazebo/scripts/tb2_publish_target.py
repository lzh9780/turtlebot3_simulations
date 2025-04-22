#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped

def main():
    rospy.init_node('tb2_target_publisher')
    pub = rospy.Publisher("/tb2/detected_object", PointStamped, queue_size=10)
    rate = rospy.Rate(1)  # 每秒发布一次目标位置

    while not rospy.is_shutdown():
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.point.x = -1.5
        msg.point.y = -1.5
        msg.point.z = 0.0
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
