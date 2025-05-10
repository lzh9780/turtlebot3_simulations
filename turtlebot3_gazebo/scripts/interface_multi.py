#!/usr/bin/env python

import rospy
import tf
import math 
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import LinkStates
from path_finding import path_generate, coord_trans, load_cost_map
from threading import Thread, Event
from cv_bridge import CvBridge
import time
import numpy as np
import cv2
from scipy.optimize import least_squares

ACTION_EXPLOR = "explor"
ACTION_EXPLOR_AGAIN = "explor_again"
ACTION_PICK_UP = "pick_up"
ACTION_SUBMIT = "submit"

STATUS_PLANNING = "planning"
STATUS_WAIT = "wait"
STATUS_COMPLETE = "complete"
STATUS_MOVING_TO = "moving"
STATUS_ARRIVE = "arrive"
STATUS_REACHING = "reaching"

STATUS_FOUND = "found"
STATUS_SELECTED = "selected"
STATUS_PICKED = "picked"
STATUS_SUBMITTED = "submitted"

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
        self.other_robot_pose = (0, 0, 0)
        self.goal = (0, 0)
        self.action = []
        self.status = STATUS_WAIT
        self.cost_map = map
        
        self.inside_area = PUBLIC_AREA
        self.area_range = [[-1.0, 1.0, -2.0, -0.5], [1.0, 2.0, -2.0, -0.75], [-2.0, -1.0, -2.0, -0.5], [-2.0, -1.0, 0.5, 2.0]]
        
        self.camera_info = CameraInfo()
        
        self.pose_sub = rospy.Subscriber(f"/{self.robot_name}/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.image_sub = rospy.Subscriber(f"/{self.robot_name}/camera/rgb/image_raw", Image, self.image_callback)
        self.info_sub = rospy.Subscriber(f"/{self.robot_name}/camera/rgb/camera_info", CameraInfo, self.camera_info_callback)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)
        
        self.vel_pub = rospy.Publisher(f"/{self.robot_name}/cmd_vel", Twist, queue_size=10)
        self.twist = Twist()
        
        # self.img_sub = rospy.Subscriber("", Image, self.image_callback, queue_size=10)
        self.image = None
        
        self.timer = time.time()
        
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
    
    def pose_callback(self, msg:PoseWithCovarianceStamped):
        # self.curr_pose = msg.pose.pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        self.curr_pose = (x, y, yaw)
        
        for i in range(len(self.area_range)):
            if self.area_range[0] < x <self.area_range[1] and self.area_range[2] < y < self.area_range[3]:
                self.inside_area = i
                break
            if i == 3:
                self.inside_area = PUBLIC_AREA
    
    def camera_info_callback(self, msg:CameraInfo):
        self.camera_info = msg
        
    def set_action(self, action):
        self.action = action
        self._new_action_event.set()
        
    def execute(self, join=False):
        t = Thread(target=self.run)
        t.start()
        if join:
            t.join()
    
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
                    start_time = time.time()
                else: curr_action = [0, self.curr_pose[2]]
            
            # exit thread when robot reach goal position
            if coord_trans(self.curr_pose[0], self.curr_pose[1]) == coord_trans(self.goal[0], self.goal[1]):
                self.twist_pub(0.0, 0.0)
                return
    
    def move_to_position(self, pose):
        if self.get_status() == STATUS_PLANNING:
            self.set_goal(pose)
            self.action_generate()
            self.set_status(STATUS_MOVING_TO)
            self.timer = time.time() 
        elif self.get_status() == STATUS_MOVING_TO:
            if self.get_pose()[:2] == self.get_goal():
                self.set_status(STATUS_ARRIVE)
                return
            if time.time() - self.timer > 3:
                self.set_stop()
                self.action_generate()
                self.timer = time.time()
    
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
    
    def detection(self, cube_list, area_list, marker_length):
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
                    cube_dict[marker_id] = []
                cube_dict[marker_id].append(tvecs[i])


            for id in cube_dict.keys():
                tvec_list = cube_dict[id]
                
                trans, _ = self.detect_cube_position(tvec_list, cube_size=0.05)
                if trans is not None:
                    rob_x_global = self.curr_pose[0]
                    rob_y_global = self.curr_pose[1]
                    rob_rot_rad = self.curr_pose[2]

                    x_cam = trans[0]
                    z_cam = trans[2]

                    x_base = z_cam + 0.076
                    y_base = -x_cam + 0

                    obj_x_global = rob_x_global + x_base * math.cos(rob_rot_rad) - y_base * math.sin(rob_rot_rad)
                    obj_y_global = rob_y_global + x_base * math.sin(rob_rot_rad) + y_base * math.cos(rob_rot_rad)
                    
                    area = -1
                    for i in range(1, 4):
                        if obj_x_global in range(self.area_range[i][0], self.area_range[i][1]) and \
                        range(self.area_range[i][2], self.area_range[i][3]):
                            area = i
                            break
                    
                    if marker_length == 0.045:
                        try:
                            cube_list[id]["pose"] = (obj_x_global, obj_y_global)
                            if cube_list[id]["area"] != area:
                                area_list[cube_list[id]["area"]]["cube_list"].remove(id)
                                cube_list[id]["area"] = area
                                area_list[area]["cube_list"].append(id)
                                
                        except (KeyError, ValueError) as e:
                            cube_list[id] = {"status": STATUS_FOUND, "pose": (obj_x_global, obj_y_global), "area": area}
                            area_list[area]["cube_list"].append(id)
                    else:
                        id += 0.5
                        try:
                            cube_list[id]["pose"] = (obj_x_global, obj_y_global)
                            if cube_list[id]["area"] != area:
                                area_list[cube_list[id]["area"]]["cube_list"].remove(id)
                                cube_list[id]["area"] = area
                                area_list[area]["cube_list"].append(id)
                                
                        except (KeyError, ValueError) as e:
                            cube_list[id] = {"status": STATUS_FOUND, "pose": (obj_x_global, obj_y_global), "area": area}
                            area_list[area]["cube_list"].append(id)

        
    def move_toward_origin(self, x, y, distance=0.225):
        length = math.hypot(x, y)
        if length == 0:
            return (x, y)  # already at origin
        dx = distance * x / length
        dy = distance * y / length
        return (x - dx, y - dy)

    def tb_action(self, area_list, cube_list):
        while True:
            mission = self.get_mission()
            if self.inside_area == SUBMISSION_AREA:
                self.detection(cube_list, area_list, 0.09)
            else: 
                self.detection(cube_list, area_list, 0.045)

            if mission == None:
                continue
            elif mission[0] is ACTION_EXPLOR:
                self.explore(area_list[mission[1]])
            elif mission[0] is ACTION_PICK_UP:
                self.pick_up(cube_list)
            elif mission[0] is ACTION_SUBMIT:
                self.submit()
            else: continue
        
    def action_generate(self):
        self.action = path_generate(self.cost_map,
            coord_trans(self.get_pose()[0], self.get_pose()[1]),
            coord_trans(self.get_goal()[0], self.get_goal()[1]),
            True, self.other_robot_pose[0], self.other_robot_pose[1])
        
        self.execute()
    
    def explore(self, area_info):
        self.move_to_position(area_info["enter_pos"])
        if self.get_status() == STATUS_ARRIVE:
            if self.curr_mission[1] == SUBMISSION_AREA or self.curr_mission[1] == STORE_AREA_3:
                self.action.append((0, 1.57), (0, -1.57))
            elif self.curr_mission[1] == STORE_AREA_1 or self.curr_mission[1] == STORE_AREA_2:
                self.action.append((0, 0), (0, 3.1))

            self.execute(True)
            self.set_status(STATUS_COMPLETE)
        elif self.get_status() == STATUS_COMPLETE:
            if len(self.action) == 0:
                self.mission_complete()
                self.set_stop()
    
    def pick_up(self, cube_list):
        # self.move_to_position(area_list[cube_list[self.get_mission()[1]]["area"]]["enter_pos"])
        # if self.get_status() == STATUS_PLANNING:
        id = self.get_mission()[1]
        target = cube_list[id]
        if target["status"] == STATUS_SELECTED:
            x = target["pose"][0] - self.curr_pose[0]
            y = target["pose"][1] - self.curr_pose[1]
            self.move_to_position(self.move_toward_origin(x + self.curr_pose[0], y + self.curr_pose[1]))
            if self.status == STATUS_ARRIVE:
                target["status"] = STATUS_PICKED
        elif target["status"] == STATUS_PICKED:
            # index = np.where(np.array(area_list[SUBMISSION_AREA]["cube_number"]) == id)[0][0]
            self.set_mission(ACTION_SUBMIT, id + 0.5)
            self.set_status(STATUS_PLANNING)
            self.submit(cube_list)
    
    def submit(self, cube_list):
        # self.move_to_position(area_list[SUBMISSION_AREA]["enter_pos"])
        id = self.get_mission()[1]
        target = cube_list[id]
        x = target["pose"][0] - self.curr_pose[0]
        y = target["pose"][1] - self.curr_pose[1]
        self.move_to_position(self.move_toward_origin(x + self.curr_pose[0], y + self.curr_pose[1]))
        if self.get_status() == STATUS_ARRIVE:
            self.set_mission(None)
            self.set_status(STATUS_WAIT)
            cube_list[id - 0.5]["status"] = STATUS_SUBMITTED
    
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


if __name__ == '__main__':
    rospy.init_node('control_interface')
    interface = Interface()
    
    interface.main()
    
    # rospy.spin()
