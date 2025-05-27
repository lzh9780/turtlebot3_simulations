#!/usr/bin/env python

import rospy
import tf
import math 
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CameraInfo
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from cv_bridge import CvBridge
import time
import numpy as np
import cv2
from scipy.optimize import least_squares
from status import *
import argparse
import json

class TurtlebotController:
    def __init__(self, name):
        self.robot_name = name
        self.curr_mission = None
        self.curr_pose = None
        self.goal = None
        self.status = STATUS_WAIT
        
        self.inside_area = PUBLIC_AREA
        self.area_range = [[-1.0, 1.0, -2.0, -0.5], [1.0, 2.0, -2.0, -0.75], [-2.0, -1.0, -2.0, -0.5], [-2.0, -1.0, 0.5, 2.0]]
        
        self.camera_info = CameraInfo()
        self.image = None
        self.goal_status = None
        
        self.status_pub = rospy.Publisher(f"/tb3/info", String, queue_size=10)
        
        self.amcl_pose_sub = rospy.Subscriber(f"{self.robot_name}/amcl_pose", PoseWithCovarianceStamped, self.pose_callback, queue_size=10)
        self.image_sub = rospy.Subscriber(f"{self.robot_name}/camera/rgb/image_raw", Image, self.image_callback, queue_size=10)
        self.info_sub = rospy.Subscriber(f"{self.robot_name}/camera/rgb/camera_info", CameraInfo, self.camera_info_callback, queue_size=10)
        self.goal_status_sub = rospy.Subscriber(f"{self.robot_name}/move_base/status", GoalStatusArray, self.goal_status_callback, queue_size=10)
        
        # self.pose_sub = rospy.Subscriber("/poses", PoseStamped, self.multi_pose_callback, queue_size=10)
        self.mission_sub = rospy.Subscriber("missions", String, self.mission_callback, queue_size=10)
        
        self.goal_pub = rospy.Publisher(f"{self.robot_name}/move_base/goal", MoveBaseActionGoal, queue_size=10)
        self.vel_pub = rospy.Publisher(f"{self.robot_name}/cmd_vel", Twist, queue_size=10)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)
        
        self.cube_pose = {}

    def twist_pub(self, vx, vyaw):
        t = Twist()
        t.linear.x = vx
        t.angular.z = vyaw
        self.vel_pub.publish(t)
    
    def image_callback(self, msg:Image):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    
    def pose_callback(self, msg:PoseWithCovarianceStamped):
        # self.curr_pose = msg.pose.pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        orientation = msg.pose.pose.orientation
        # (_, _, yaw) = tf.transformations.euler_from_quaternion(
        #     [orientation.x, orientation.y, orientation.z, orientation.w]
        # )
        
        self.curr_pose = (x, y, z, orientation.x, orientation.y, orientation.z, orientation.w)
        
        for i in range(len(self.area_range)):
            if self.area_range[i][0] < x and x <self.area_range[i][1] and self.area_range[i][2] < y and y < self.area_range[i][3]:
                self.inside_area = i
                break
            if i == 3:
                self.inside_area = PUBLIC_AREA
    
    def camera_info_callback(self, msg:CameraInfo):
        self.camera_info = msg
    
    def mission_callback(self, msg:String):
        d = json.loads(msg.data)
        try:
            if d["id"] == self.robot_name and d["mission"] != self.curr_mission:
                self.set_mission(d["mission"])
                self.set_goal(d["goal"])
                self.set_status(STATUS_PLANNING)
        
        except KeyError:
            pass
    
    def goal_status_callback(self, msg:GoalStatusArray):
        if len(msg.status_list) > 0:
            l = msg.status_list[::-1]
            self.goal_status = l[0]
        else:
            self.goal_status = None
    
    def detect_cube_position(self, tvecs, cube_size):
        L = cube_size / 2
        local_positions = [
            np.array([0, 0, L]),
            np.array([L, 0, 0]),
            np.array([0, L, 0]),
            np.array([0, 0, -L]),
            np.array([-L, 0, 0]),
            np.array([0, -L, 0])
        ]

        def residual(params, observations):
            R = params[:9].reshape(3, 3)
            T = params[9:12]
            residuals = []
            for i, P_obs in enumerate(observations):
                local_idx = i % 6
                P_local = local_positions[local_idx]
                P_pred = R @ P_local + T
                residuals.extend(P_pred - P_obs)
            return residuals

        observations = [tvec[0] for tvec in tvecs]
        initial_params = np.eye(3).flatten().tolist() + [0.0, 0.0, 0.0]

        result = least_squares(
            residual,
            initial_params,
            args=(observations,),
            method='trf',
            max_nfev=200
        )

        R_optimized = result.x[:9].reshape(3, 3)
        T_optimized = result.x[9:12]

        U, _, Vt = np.linalg.svd(R_optimized)
        R_optimized = U @ Vt

        return T_optimized, R_optimized
    
    def detection(self, marker_length):
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        _, frame = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
        # detect ArUco marker
        corners, ids, _ = self.detector.detectMarkers(frame)
        self.cube_pose = {}
        
        if ids is not None and self.camera_info.K is not None and self.camera_info.D is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_length, np.reshape(self.camera_info.K, (3, 3)), self.camera_info.D
            )

            cube_dict = {}  # cube_idx -> list of tvec
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in cube_dict:
                    cube_dict[marker_id] = []
                cube_dict[marker_id].append(tvecs[i])
            
            for marker_id in cube_dict.keys():
                tvec_list = cube_dict[marker_id]
                
                trans, _ = self.detect_cube_position(tvec_list, cube_size=0.05)
                if trans is not None:
                    rob_x_global = self.curr_pose[0]
                    rob_y_global = self.curr_pose[1]
                    
                    (_, _, yaw) = tf.transformations.euler_from_quaternion(
                        [self.curr_pose[3], self.curr_pose[4], self.curr_pose[5], self.curr_pose[6]])
                    rob_rot_rad = yaw

                    x_cam = trans[0]
                    z_cam = trans[2]

                    x_base = z_cam + 0.076
                    y_base = -x_cam + 0
                    
                    self.cube_pose[marker_id] = (x_base, y_base)

                    obj_x_global = rob_x_global + x_base * math.cos(rob_rot_rad) - y_base * math.sin(rob_rot_rad)
                    obj_y_global = rob_y_global + x_base * math.sin(rob_rot_rad) + y_base * math.cos(rob_rot_rad)
                    
                    area = -1
                    for i in range(1, 4):
                        if obj_x_global > self.area_range[i][0] and obj_x_global < self.area_range[i][1] and \
                        obj_y_global > self.area_range[i][2] and obj_y_global < self.area_range[i][3]:
                            area = i
                            break
                    
                    s = String()
                    if marker_length == 0.045:
                        s.data = json.dumps({"type": "cube", "id": int(marker_id), "info": {"pose": (obj_x_global, obj_y_global, 0, 0, 0, 0, 1), "area": area, "status": STATUS_FOUND}})
                    elif marker_length == 0.09:
                        s.data = json.dumps({"type": "marker", "id": int(marker_id), "info": {"pose": (obj_x_global, obj_y_global, 0, 0, 0, 1, 0), "status": STATUS_FOUND}})

                    self.status_pub.publish(s)
        
    def move_to_position(self):
        if self.goal == None: 
            return 
        
        action_goal = MoveBaseActionGoal()
        action_goal.header.frame_id = "map"
        action_goal.header.stamp = rospy.Time.now()
        action_goal.goal.target_pose.header.frame_id = "map"
        action_goal.goal.target_pose.header.stamp = rospy.Time.now()
        action_goal.goal.target_pose.pose.position.x = self.goal[0]
        action_goal.goal.target_pose.pose.position.y = self.goal[1]
        action_goal.goal.target_pose.pose.orientation.z = self.goal[5]
        action_goal.goal.target_pose.pose.orientation.w = self.goal[6]
    
        self.goal_pub.publish(action_goal)
        
        if self.goal_status != None and self.goal_status.status == 1:
                self.set_status(STATUS_MOVING_TO)
    
    def move_toward_origin(self, x, y, distance=0.225):
        length = math.hypot(x, y)
        if length == 0:
            return (x, y)  # already at origin
        dx = distance * x / length
        dy = distance * y / length
        return (x - dx, y - dy)

    def mission_complete(self):
        if self.status == STATUS_COMPLETE:
            self.set_mission(None)
            self.set_status(STATUS_WAIT)
    
    def explore(self):
        if self.status == STATUS_PLANNING:
            self.move_to_position()
        elif self.status == STATUS_MOVING_TO:
            if self.goal_status != None and self.goal_status.status == 3:
                self.set_status(STATUS_ARRIVE)
            elif self.goal_status != None and self.goal_status.status != 1:
                self.move_to_position()
        elif self.status == STATUS_ARRIVE:
            start_time = time.time()
            while time.time() - start_time < 15:
                if self.curr_mission[1] == SUBMISSION_AREA:
                    self.detection(0.09)
                else:
                    self.detection(0.045)
                
                if time.time() - start_time < 5:
                    self.twist_pub(0, -0.2)
                else: 
                    self.twist_pub(0, 0.2)
            
            self.twist_pub(0, 0)
            self.set_status(STATUS_COMPLETE)
        elif self.status == STATUS_COMPLETE:
            self.mission_complete()
    
    def pick_up(self):
        # self.move_to_position()
        # if self.get_status() == STATUS_PLANNING:
        cube_id = self.curr_mission[1]
        
        if self.status == STATUS_PLANNING:
            self.move_to_position()
        elif self.status == STATUS_MOVING_TO:
            if self.goal_status != None and self.goal_status.status == 3:
                self.set_status(STATUS_ARRIVE)
            elif self.goal_status != None and self.goal_status.status != 1:
                self.move_to_position()
        elif self.status == STATUS_ARRIVE:
            # self.detection(0.045)
            # try:
            #     target_pose = self.cube_pose[cube_id]
                # x, y = self.move_toward_origin(target_pose[0], target_pose[1])
                # yaw = math.atan2(target_pose[1], target_pose[0])
                # if yaw > 0.01:
                #     self.twist_pub(0, yaw)
                #     return
                
                # if math.sqrt(target_pose[0] ** 2 + target_pose[1] ** 2) > 0.22:
                #     self.twist_pub(math.sqrt(target_pose[0] ** 2 + target_pose[1] ** 2) / 5, 0)
                #     return

                # self.twist_pub(0, 0)
                
            s = String()
            s.data = json.dumps({"type": "cube", "id": int(cube_id), "info": {"pose": None, "area": None, "status": STATUS_PICKED}})
            self.status_pub.publish(s)
            
            self.set_mission((ACTION_SUBMIT, cube_id))
            self.set_status(STATUS_PLANNING)
            self.set_goal(None)
                
            # except KeyError:
            #     print("Target not found")
            #     self.detection(0.045)
                
                            
        # if target["status"] == STATUS_SELECTED:
        #     x = target["pose"][0] - self.curr_pose[0]
        #     y = target["pose"][1] - self.curr_pose[1]
        #     self.move_to_position()
        #     if self.status == STATUS_ARRIVE:
        #         target["status"] = STATUS_PICKED
        # elif target["status"] == STATUS_PICKED:
        #     # index = np.where(np.array(area_list[SUBMISSION_AREA]["cube_number"]) == id)[0][0]
        #     self.set_mission(ACTION_SUBMIT, id + 0.5)
        #     self.set_status(STATUS_PLANNING)
        #     self.submit(cube_list)
    
    def submit(self):
        # self.move_to_position()
        cube_id = self.curr_mission[1]
        
        if self.status == STATUS_PLANNING:
            self.move_to_position()
        elif self.status == STATUS_MOVING_TO:
            if self.goal_status != None and self.goal_status.status == 3:
                self.set_status(STATUS_ARRIVE)
            elif self.goal_status != None and self.goal_status.status != 1:
                self.move_to_position()
        elif self.status == STATUS_ARRIVE:
            s = String()
            s.data = json.dumps({"type": "cube", "id": int(cube_id), "info": {"pose": None, "area": None, "status": STATUS_SUBMITTED}})
            self.status_pub.publish(s)
            
            self.set_status(STATUS_COMPLETE)
            self.mission_complete()
        
        # target = cube_list[id]
        # x = target["pose"][0] - self.curr_pose[0]
        # y = target["pose"][1] - self.curr_pose[1]
        # self.move_to_position()
        # if self.get_status() == STATUS_ARRIVE:
        #     self.set_mission(None)
        #     self.set_status(STATUS_WAIT)
        #     cube_list[id - 0.5]["status"] = STATUS_SUBMITTED

    def tb_action(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.curr_mission == None:
                pass
            elif self.curr_mission[0] == ACTION_EXPLOR:
                self.explore()
            elif self.curr_mission[0] == ACTION_PICK_UP:
                self.pick_up()
            elif self.curr_mission[0] == ACTION_SUBMIT:
                self.submit()
            else: pass
            
            s = String()
            s.data = json.dumps({"type": "robot", "id": self.robot_name, "info": {"status": self.status, "pose": self.curr_pose, "mission": self.curr_mission}})
            print(s.data)
            self.status_pub.publish(s)
            
            # if self.curr_mission[0] == ACTION_PICK_UP:
            #     pass
            # elif self.curr_mission[0] == ACTION_SUBMIT:
            #     pass
    
    def set_mission(self, mission):
        self.curr_mission = mission
    
    def set_goal(self, goal):
        self.goal = goal
    
    def set_status(self, status):
        self.status = status


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Arguments")
    parser.add_argument("robot_name", default="")
    
    args = parser.parse_args()
    print(args.robot_name)
    
    rospy.init_node(f'robot_control_{args.robot_name}')
    # interface = Interface()
    
    # interface.main()
    control = TurtlebotController(name=args.robot_name)
    control.tb_action()
    
    # rospy.spin()
