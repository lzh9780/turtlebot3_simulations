#!/usr/bin/env python

import rospy
import tf
import math 
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from path_finding import path_generate, coord_trans
from threading import Thread, Event
from cv_bridge import CvBridge
import time
import numpy as np
import cv2

class Interface:
    def __init__(self):
        self.action = []
        
        self.curr_pose = (0, 0, 0)
        self.goal = (-1, -1)
        
        self.rgb_image = Image()
        
        self.goal_sub = rospy.Subscriber("/customized_goal", Point, self.goal_callback)
        # self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.twist = Twist()
        # self.img_sub = rospy.Subscriber("", Image, self.image_callback, queue_size=10)
        
        self._stop_event = Event()
        self._new_action_event = Event()
    
    def action_generate(self):
        while coord_trans(self.curr_pose[0], self.curr_pose[1]) != coord_trans(self.goal[0], self.goal[1]):
            self.action = path_generate(coord_trans(self.curr_pose[0], self.curr_pose[1]), coord_trans(self.goal[0], self.goal[1]))
            self._new_action_event.set()
            time.sleep(5)
        
    def execute(self):
        t = Thread(target=self.run)
        t.start()
    
    def twist_pub(self, x, yaw):
        self.twist.linear.x = x
        self.twist.angular.z = yaw
        self.vel_pub.publish(self.twist)
    
    def run(self):
        start_time = time.time()
        rate = rospy.Rate(10)
        
        if len(self.action) != 0:
            curr_action = self.action.pop(0)
        else: curr_action = [0, self.curr_pose[2]]
        
        linear_run_time = curr_action[0] / 0.1
        angular_speed = 0.1 if curr_action[1] - self.curr_pose[2] > 0 else -0.1
        angular_run_time = abs((curr_action[1] - self.curr_pose[2]) / 0.1)
        
        while not self._stop_event.is_set(): 
            print(curr_action, self.curr_pose)
            print(linear_run_time, angular_speed, angular_run_time)
            if time.time() - start_time < angular_run_time:
                # execute rotation 
                self.twist_pub(0.0, angular_speed)
                rate.sleep()
            elif time.time() - start_time < angular_run_time + linear_run_time:
                # execute linear movement
                self.twist_pub(0.1, 0.0)
                rate.sleep()
            else:
                # end of last command
                if len(self.action) != 0:
                    # get new command if there is any
                    curr_action = self.action.pop(0)
                    linear_run_time = curr_action[0] / 0.1
                    angular_speed = 0.1 if curr_action[1] - self.curr_pose[2] > 0 else -0.1
                    angular_run_time = abs((curr_action[1] - self.curr_pose[2]) / 0.1)
                    start_time = time.time()
                else: 
                    # keep positon if no new command
                    curr_action = [0, self.curr_pose[2]]
                    self.twist_pub(0.0, 0.0)
                    rate.sleep()
            
            # update action list and move command
            if self._new_action_event.is_set():
                self._new_action_event.clear()
                if len(self.action) != 0:
                    curr_action = self.action.pop(0)
                else: curr_action = [0, self.curr_pose[2]]
            
            # exit thread when robot reach goal position
            if coord_trans(self.curr_pose[0], self.curr_pose[1]) == coord_trans(self.goal[0], self.goal[1]):
                self.twist_pub(0.0, 0.0)
                return
    
    # def pose_callback(self, msg:PoseWithCovarianceStamped):
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     orientation_q = msg.pose.pose.orientation
    #     quaternion = (
    #         orientation_q.x,
    #         orientation_q.y,
    #         orientation_q.z,
    #         orientation_q.w
    #     )
    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     yaw = euler[2] # math.degrees(euler[2])
    #     # print("Robot Position => x: {:.2f}, y: {:.2f}, yaw: {:.2f}°".format(x, y, yaw))
    #     self.curr_pose = (x, y, yaw)
    #     print(self.curr_pose)
    
    def goal_callback(self, msg: Point):
        self.goal = (msg.x, msg.y)
        self._stop_event.set()
        time.sleep(0.1)
        self._stop_event.clear()
        
        t = Thread(target=self.action_generate)
        t.start()
        
        self.execute()
        
    def image_callback(self, msg:Image):
        self.rgb_image = msg
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(self.rgb_image, desired_encoding='rgb8')
        # detect ArUco marker
        corners, ids, _ = detector.detectMarkers(frame)
        global_pose = {}
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_length, camera_matrix, dist_coeffs
            )

            for i in range(len(ids)):
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
                id = ids[i][0]
                # --------------------------
                # 1. Get the coordinate of the code in camera
                x_cam = tvecs[i][0][0]
                y_cam = tvecs[i][0][1]
                z_cam = tvecs[i][0][2]

                # 2. Convert the camera coordinate system to the robot base coordinate system (accounting for mounting offset)
                # Assume the camera is facing forward (same direction as the robot base)
                x_base = z_cam + CAMERA_OFFSET_X
                y_base = x_cam + CAMERA_OFFSET_Y
                                
                
                # 3. Get the robot's global position (assuming it is known)
                rob_rot_rad = 1.68    # Global orientation of the robot (angle)
                rob_x_global = 1.0    # The robot's X position in the global coordinate system
                rob_y_global = 1.5    # The robot's Y position in the global coordinate system
                

                # 4. Convert the base coordinate system to the global coordinate system
                obj_x_global = rob_x_global + x_base * cos(rob_rot_rad) - y_base * sin(rob_rot_rad)
                obj_y_global = rob_y_global + x_base * sin(rob_rot_rad) + y_base * cos(rob_rot_rad)
                
                global_pose[id] = (obj_x_global, obj_y_global)

if __name__ == '__main__':
    rospy.init_node('control_interface')
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    interface = Interface()
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            x = trans[0]
            y = trans[1]
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            # yaw_deg = math.degrees(yaw)
            # print("x: {:.2f}, y: {:.2f}, yaw: {:.2f}°".format(x, y, yaw_deg))
            interface.curr_pose = (x, y, yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()