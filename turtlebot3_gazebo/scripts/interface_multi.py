#!/usr/bin/env python

import rospy
import tf
import math 
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import LinkStates
from path_finding import path_generate, coord_trans, load_cost_map
from threading import Thread, Event
from cv_bridge import CvBridge
import time
import numpy as np
import cv2

ACTION_EXPLOR = "explor"
ACTION_EXPLOR_AGAIN = "explor_again"
ACTION_PICK_UP = "pick_up"
ACTION_SUBMIT = "submit"

STATUS_WAIT = "wait"
STATUS_COMPLETE = "complete"
STATUS_MOVING_TO = "moving_to"
STATUS_ARRIVE = "arrive"
STATUS_PLANNING = "planning"

SUBMISSION_AREA = 0
STORE_AREA_1 = 1
STORE_AREA_2 = 2
STORE_AREA_3 = 3
PUBLIC_AREA = 4

class TurtlebotController:
    def __init__(self, name, map):
        self.robot_name = name
        self.curr_mission = None
        self.curr_pose = (0, 0, 0)
        self.goal = (0, 0)
        self.action = []
        self.status = STATUS_WAIT
        self.cost_map = map
        
        self.inside_area = PUBLIC_AREA
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)
        
        self.vel_pub = rospy.Publisher("/{}/cmd_vel".format(self.robot_name), Twist, queue_size=10)
        self.twist = Twist()
        
        # self.img_sub = rospy.Subscriber("", Image, self.image_callback, queue_size=10)
        self.image = None
        
        self._stop_event = Event()
        self._stop_event.clear()
        self._new_action_event = Event()
    
    def mission_complete(self):
        if self.status == STATUS_COMPLETE:
            self.set_mission(None)
            self.set_status(STATUS_WAIT)
    
    def twist_pub(self, x, yaw):
        self.twist.linear.x = x
        self.twist.angular.z = yaw
        self.vel_pub.publish(self.twist)
    
    def image_callback(self, msg:Image):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        
        # detect ArUco marker
        # corners, ids, _ = self.detector.detectMarkers(self.image)
        # global_pose = {}
        
        # if ids is not None:
        #     rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        #         corners, marker_length, camera_matrix, dist_coeffs
        #     )

        #     for i in range(len(ids)):
        #         id = ids[i][0]
        #         # --------------------------
        #         # 1. Get the coordinate of the code in camera
        #         x_cam = tvecs[i][0][0]
        #         y_cam = tvecs[i][0][1]
        #         z_cam = tvecs[i][0][2]

        #         # 2. Convert the camera coordinate system to the robot base coordinate system (accounting for mounting offset)
        #         # Assume the camera is facing forward (same direction as the robot base)
        #         x_base = z_cam + CAMERA_OFFSET_X
        #         y_base = x_cam + CAMERA_OFFSET_Y
                                
                
        #         # 3. Get the robot's global position (assuming it is known)
        #         rob_rot_rad = 1.68    # Global orientation of the robot (angle)
        #         rob_x_global = 1.0    # The robot's X position in the global coordinate system
        #         rob_y_global = 1.5    # The robot's Y position in the global coordinate system
                

        #         # 4. Convert the base coordinate system to the global coordinate system
        #         obj_x_global = rob_x_global + x_base * math.cos(rob_rot_rad) - y_base * math.sin(rob_rot_rad)
        #         obj_y_global = rob_y_global + x_base * math.sin(rob_rot_rad) + y_base * math.cos(rob_rot_rad)
                
        #         global_pose[id] = (obj_x_global, obj_y_global)
        
    def set_action(self, action):
        self.action = action
        self._new_action_event.set()
        
    def execute(self):
        t = Thread(target=self.run)
        t.start()
    
    def run(self):
        start_time = time.time()
        rate = rospy.Rate(10)
        
        if len(self.action) != 0:
            curr_action = self.action.pop(0)
        else: curr_action = [0, self.curr_pose[2]]
        
        linear_run_time = curr_action[0] / 0.1
        print(curr_action, self.curr_pose)
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
    
    def explore(self):
        pass
    
    def pick_up(self):
        pass
    
    def submit(self):
        pass
    
    def get_mission(self):
        return self.curr_mission
    
    def set_mission(self, mission, area):
        self.curr_mission = (mission, area)
    
    def get_goal(self):
        return self.goal
    
    def set_goal(self, goal):
        self.goal = goal
        
    def get_pose(self):
        return self.curr_pose
    
    def get_status(self):
        return self.status
    
    def set_status(self, status):
        self.status = status
    
    def set_stop(self):
        self._stop_event.set()
        time.sleep(0.1)
        self._stop_event.clear()
            

class Interface:
    def __init__(self):
        self.cost_map = load_cost_map()
        
        self.tb3_1 = TurtlebotController("tb3_1", self.cost_map)
        self.tb3_2 = TurtlebotController("tb3_2", self.cost_map)
        
        # self.goal_sub = rospy.Subscriber("/customized_goal", Point, self.goal_callback)
        # self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        
        self.pose_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.pose_callback)
        
        self.cube_position = {}
        self.target_list = []
        self.area_entre_pos = [(0.0, 1.5), (1.575, 1), (-1.25, 0.5), (-1, -1.25)]
    
    def pose_callback(self, msg:LinkStates):
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
        counter = 0
        for name in msg.name:
            if name == "turtlebot3_1::base_footprint":
                pose:Pose = msg.pose[counter]
                quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                )
                yaw = tf.transformations.euler_from_quaternion(quaternion)
                self.tb3_1.curr_pose = (pose.position.x, pose.position.y, yaw[2])
            elif name == "turtlebot3_2::base_footprint":
                pose:Pose = msg.pose[counter]
                quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                )
                yaw = tf.transformations.euler_from_quaternion(quaternion)
                self.tb3_2.curr_pose = (pose.position.x, pose.position.y, yaw[2])

            counter += 1
    
    def mission_start(self):
        t1 = Thread(target=self.tb_action, args=(self.tb3_1, self.tb3_2))
        t1.start()
        
        t2 = Thread(target=self.tb_action, args=(self.tb3_2, self.tb3_1))
        t2.start()
    
    def tb_action(self, robot: TurtlebotController, other_robot: TurtlebotController):
        timer = time.time()
        while not rospy.is_shutdown():
            mission = robot.get_mission()
            if mission != None and mission[0] == ACTION_EXPLOR:
                if robot.get_status() == STATUS_PLANNING:
                    robot.set_goal(self.area_entre_pos[mission[1]])
                    self.action_generate(robot, other_robot)
                    robot.set_status(STATUS_MOVING_TO)
                    timer = time.time()
                elif robot.get_status() == STATUS_MOVING_TO:
                    if robot.get_pose()[:2] == robot.get_goal():
                        robot.set_status(STATUS_ARRIVE)
                        continue
                    if time.time() - timer > 3:
                        robot.set_stop()
                        self.action_generate(robot, other_robot)
                        timer = time.time()
                elif robot.get_status() == STATUS_ARRIVE:
                    if mission[1] == SUBMISSION_AREA or mission[1] == STORE_AREA_3:
                        robot.action.append((0, 1.57), (0, -1.57))
                    elif mission[1] == STORE_AREA_1 or mission[1] == STORE_AREA_2:
                        robot.action.append((0, 0), (0, 3.1))
                    robot.set_status = STATUS_COMPLETE
                    robot.execute()
                elif robot.get_status() == STATUS_COMPLETE:
                    if len(robot.action) == 0:
                        robot.mission_complete()
                        robot.set_stop()
                
        
    def action_generate(self, robot: TurtlebotController, other_robot: TurtlebotController):
        robot.action = path_generate(robot.cost_map,
            coord_trans(robot.get_pose()[0], -robot.get_pose()[1]),
            coord_trans(robot.get_goal()[0], robot.get_goal()[1]),
            True, other_robot.get_pose()[0], other_robot.get_pose()[1])
        
        robot.execute()
        
    def get_distance(self, robot:TurtlebotController, target_pos):
        robot_pose = robot.get_pose()
        return math.sqrt((robot_pose[0] - target_pos[0]) ** 2 + (robot_pose[1] - target_pos[1]) ** 2)
    
    def compare_distance(self, area):
        if self.get_distance(self.tb3_1, self.area_entre_pos[area]) \
            <= self.get_distance(self.tb3_2, self.area_entre_pos[area]):
            return self.tb3_1
        else: return self.tb3_2

    def robot_free(self, robot:TurtlebotController):
        if robot.get_status() == STATUS_WAIT:
            return True
        else: return False
    
    def duplicate_mission(self, mission, area):
        return (self.tb3_1.get_mission() == (mission, area) or \
            self.tb3_2.get_mission == (mission, area))
        
    def mission_pub(self, mission, area):
        if not self.duplicate_mission(mission, area):
            if self.robot_free(self.tb3_1) and self.robot_free(self.tb3_2):
                robot = self.compare_distance(area)
                robot.set_mission(mission, area)
                robot.set_status(STATUS_PLANNING)
            elif self.robot_free(self.tb3_1): 
                self.tb3_1.set_mission(mission, area)
                self.tb3_1.set_status(STATUS_PLANNING)
            elif self.robot_free(self.tb3_2): 
                self.tb3_2.set_mission(mission, area)
                self.tb3_2.set_status(STATUS_PLANNING)
            else: return False
        else: return False
        
        return True
    
    def main(self):
        self.mission_start()
        while not rospy.is_shutdown():
            print("First robot status:", self.tb3_1.curr_pose, self.tb3_1.action)
            print("Second robot status:", self.tb3_2.curr_pose, self.tb3_2.action)
            if len(self.target_list) != 3:
                self.mission_pub(ACTION_EXPLOR, SUBMISSION_AREA)
            
            if len(self.cube_position) != 3:
                for i in range(1, 4):
                    if i not in self.cube_position:
                        mission_published = self.mission_pub(ACTION_EXPLOR, i)
                        if mission_published:
                            self.cube_position[i] = {}
            elif sum(len(self.cube_position[i]) for i in self.cube_position.keys()) != 6:
                for i in range(1, 4):
                    if len(self.cube_position[i]) < 3:
                        self.mission_pub(ACTION_EXPLOR_AGAIN, i)


if __name__ == '__main__':
    rospy.init_node('control_interface')
    interface = Interface()
    
    interface.main()
    
    # rospy.spin()
