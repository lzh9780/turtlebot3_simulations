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
        self.detected_cubes = {}
        
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

    def twist_pub(self, vx, vyaw):
        t = Twist()
        t.linear.x = vx
        t.angular.z = vyaw
        self.vel_pub.publish(t)
    
    def image_callback(self, msg:Image):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        if self.curr_mission == None:
            return
        
        self.detection(0.09 if self.curr_mission[1] == SUBMISSION_AREA else 0.045)
        if self.status == STATUS_ARRIVE:
            for marker_id in self.detected_cubes:
                s = String()
                if self.detected_cubes[marker_id]["area"] != SUBMISSION_AREA:
                    s.data = json.dumps({"type": "cube", "id": int(marker_id), 
                                         "info": {"pose": (self.detected_cubes[marker_id]["pose"][0], 
                                                           self.detected_cubes[marker_id]["pose"][1], 0, 0, 0, 0, 1), 
                                                  "area": self.detected_cubes[marker_id]["area"], 
                                                  "status": STATUS_FOUND}})
                else:
                    s.data = json.dumps({"type": "marker", "id": int(marker_id), 
                                         "info": {"pose": (self.detected_cubes[marker_id]["pose"][0], 
                                                           self.detected_cubes[marker_id]["pose"][1], 0, 0, 0, 0, 1), 
                                                  "status": STATUS_FOUND}})

                self.status_pub.publish(s)
    
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
        
        if ids is not None and self.camera_info.K is not None and self.camera_info.D is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_length, np.reshape(self.camera_info.K, (3, 3)), self.camera_info.D
            )

            cube_dict = {}  # cube_idx -> list of tvec
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in cube_dict:
                    cube_dict[marker_id] = {"tvecs": [], "pose": None, "area": -1}
                cube_dict[marker_id]["tvecs"].append(tvecs[i])
            
            for marker_id in self.detected_cubes.keys():
                tvec_list = self.detected_cubes[marker_id]["tvecs"]
                trans, _ = self.detect_cube_position(tvec_list, cube_size=0.05)
                
                rob_x_global = self.curr_pose[0]
                rob_y_global = self.curr_pose[1]
                
                (_, _, yaw) = tf.transformations.euler_from_quaternion(
                    [self.curr_pose[3], self.curr_pose[4], self.curr_pose[5], self.curr_pose[6]])
                rob_rot_rad = yaw

                x_cam = trans[0]
                z_cam = trans[2]

                x_base = z_cam + 0.076
                y_base = -x_cam + 0

                obj_x_global = rob_x_global + x_base * math.cos(rob_rot_rad) - y_base * math.sin(rob_rot_rad)
                obj_y_global = rob_y_global + x_base * math.sin(rob_rot_rad) + y_base * math.cos(rob_rot_rad)
                    
                self.detected_cubes[marker_id]["pose"] = (obj_x_global, obj_y_global)
                
                area = -1
                for i in range(1, 4):
                    if obj_x_global > self.area_range[i][0] and obj_x_global < self.area_range[i][1] and \
                    obj_y_global > self.area_range[i][2] and obj_y_global < self.area_range[i][3]:
                        area = i
                        break
                self.detected_cubes[marker_id]["area"] = area
                
            self.detected_cubes = cube_dict
        else: 
            self.detected_cubes = {}
        
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
    
    # def move_toward_origin(self, x, y, distance=0.225):
    #     length = math.hypot(x, y)
    #     if length == 0:
    #         return (x, y)  # already at origin
    #     dx = distance * x / length
    #     dy = distance * y / length
    #     return (x - dx, y - dy)

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
        elif self.status == STATUS_ARRIVE:
            self.twist_pub(0, -0.1)
            time.sleep(8)
            self.twist_pub(0, 0.1)
            time.sleep(16)
            self.twist_pub(0, 0)
            self.set_status(STATUS_COMPLETE)
        elif self.status == STATUS_COMPLETE:
            self.mission_complete()
    
    # def pick_up(self):
    #     # self.move_to_position()
    #     # if self.get_status() == STATUS_PLANNING:
    #     id = self.get_mission()[1]
        
    #     if target["status"] == STATUS_SELECTED:
    #         x = target["pose"][0] - self.curr_pose[0]
    #         y = target["pose"][1] - self.curr_pose[1]
    #         self.move_to_position()
    #         if self.status == STATUS_ARRIVE:
    #             target["status"] = STATUS_PICKED
    #     elif target["status"] == STATUS_PICKED:
    #         # index = np.where(np.array(area_list[SUBMISSION_AREA]["cube_number"]) == id)[0][0]
    #         self.set_mission(ACTION_SUBMIT, id + 0.5)
    #         self.set_status(STATUS_PLANNING)
    #         self.submit(cube_list)
    
    # def submit(self):
    #     # self.move_to_position()
    #     id = self.get_mission()[1]
    #     target = cube_list[id]
    #     x = target["pose"][0] - self.curr_pose[0]
    #     y = target["pose"][1] - self.curr_pose[1]
    #     self.move_to_position()
    #     if self.get_status() == STATUS_ARRIVE:
    #         self.set_mission(None)
    #         self.set_status(STATUS_WAIT)
    #         cube_list[id - 0.5]["status"] = STATUS_SUBMITTED

    def tb_action(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.inside_area == SUBMISSION_AREA:
                self.detection(0.09)
            else:
                self.detection(0.045)

            if self.curr_mission == None:
                pass
            elif self.curr_mission[0] == ACTION_EXPLOR:
                self.explore()
            # elif self.curr_mission[0] == ACTION_PICK_UP:
            #     self.pick_up()
            # elif self.curr_mission[0] == ACTION_SUBMIT:
            #     self.submit()
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
        

'''
class Interface:
    def __init__(self):
        self.cost_map = load_cost_map()
        
        self.tb3_1 = TurtlebotController("tb3_1", self.cost_map)
        self.tb3_2 = TurtlebotController("tb3_2", self.cost_map)
        
        # self.goal_sub = rospy.Subscriber("/customized_goal", Point, self.goal_callback)
        # self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        
        # self.pose_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.pose_callback)
        
        self.area_list = {}
        self.cube_list = {}
        # self.target_list = []
        # self.area_entre_pos = [(0.0, 1.5), (1.575, 1), (-1.25, 0.5), (-1, -1.25)]
        
        self.area_list[SUBMISSION_AREA] = {"enter_pos": (0.0, -1.5), "cube_number":-1, "cube_list": []}
        self.area_list[STORE_AREA_1] = {"enter_pos": (1.625, -1), "cube_number":-1, "cube_list": []}
        self.area_list[STORE_AREA_2] = {"enter_pos": (-1.25, -0.75), "cube_number":-1, "cube_list": []}
        self.area_list[STORE_AREA_3] = {"enter_pos": (-1.25, 1.25), "cube_number":-1, "cube_list": []}
        
        # self.cube_list[id] = {"status": str, "pose": tuple, "area": int}
    
    # def pose_callback(self, msg:LinkStates):
    # #     x = msg.pose.pose.position.x
    # #     y = msg.pose.pose.position.y
    # #     orientation_q = msg.pose.pose.orientation
    # #     quaternion = (
    # #         orientation_q.x,
    # #         orientation_q.y,
    # #         orientation_q.z,
    # #         orientation_q.w
    # #     )
    # #     euler = tf.transformations.euler_from_quaternion(quaternion)
    # #     yaw = euler[2] # math.degrees(euler[2])
    # #     # print("Robot Position => x: {:.2f}, y: {:.2f}, yaw: {:.2f}°".format(x, y, yaw))
    # #     self.curr_pose = (x, y, yaw)
    # #     print(self.curr_pose)
    #     counter = 0
    #     for name in msg.name:
    #         if name == "turtlebot3_1::base_footprint":
    #             pose:Pose = msg.pose[counter]
    #             quaternion = (
    #                 pose.orientation.x,
    #                 pose.orientation.y,
    #                 pose.orientation.z,
    #                 pose.orientation.w
    #             )
    #             rot = tf.transformations.euler_from_quaternion(quaternion)
    #             self.tb3_1.curr_pose = (pose.position.x, pose.position.y, rot[2])
    #             self.tb3_2.other_robot_pose = (pose.position.x, pose.position.y, rot[2])
    #         elif name == "turtlebot3_2::base_footprint":
    #             pose:Pose = msg.pose[counter]
    #             quaternion = (
    #                 pose.orientation.x,
    #                 pose.orientation.y,
    #                 pose.orientation.z,
    #                 pose.orientation.w
    #             )
    #             rot = tf.transformations.euler_from_quaternion(quaternion)
    #             self.tb3_2.curr_pose = (pose.position.x, pose.position.y, rot[2])
    #             self.tb3_1.other_robot_pose = (pose.position.x, pose.position.y, rot[2])

    #         counter += 1
    
    def mission_start(self):
        self.t1 = Thread(target=self.tb3_1.tb_action, args=(self.area_list, self.cube_list))
        self.t1.start()
        
        self.t2 = Thread(target=self.tb3_2.tb_action, args=(self.area_list, self.cube_list))
        self.t2.start()

    def get_distance(self, robot:TurtlebotController, target_pos):
        robot_pose = robot.get_pose()
        return math.sqrt((robot_pose[0] - target_pos[0]) ** 2 + (robot_pose[1] - target_pos[1]) ** 2)
    
    def compare_distance(self, area):
        if self.get_distance(self.tb3_1, self.area_list[area]["enter_pos"]) \
            <= self.get_distance(self.tb3_2, self.area_list[area]["enter_pos"]):
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
            print(f"{self.tb3_1.robot_name} status:", self.tb3_1.get_pose(), self.tb3_1.get_mission(), self.tb3_1.action)
            print(f"{self.tb3_2.robot_name} status:", self.tb3_2.get_pose(), self.tb3_2.get_mission(), self.tb3_2.action)
            
            self.tb3_1.other_robot_pose = self.tb3_2.get_pose()
            self.tb3_2.other_robot_pose = self.tb3_1.get_pose()
            
            if self.area_list[SUBMISSION_AREA]["cube_number"] != 3:
                self.mission_pub(ACTION_EXPLOR, SUBMISSION_AREA)
            else:
                for i in self.area_list[SUBMISSION_AREA]["cube_list"]:
                    try:
                        if self.cube_list[i]["status"] == STATUS_FOUND:
                            mission_published = self.mission_pub(ACTION_PICK_UP, i)
                            if mission_published:
                                self.cube_list[i]["status"] = STATUS_SELECTED
                    except KeyError:
                        continue
            
            for i in range(1, 4):
                if self.area_list[i]["cube_number"] == -1:
                    mission_published = self.mission_pub(ACTION_EXPLOR, i)
                    if mission_published:
                        self.area_list[i]["cube_number"] = 0
                elif len(self.cube_list) != 6 and self.area_list[i]["cube_number"] < 3:
                    self.mission_pub(ACTION_EXPLOR, i)
'''

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
