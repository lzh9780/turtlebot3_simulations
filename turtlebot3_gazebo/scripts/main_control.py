#!/usr/bin/env python

from status import *
from path_finding import path_generate, coord_trans, load_cost_map
import math
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from std_msgs.msg import Header, String
import time

class MainControl:
    def __init__(self):
        self.cost_map = load_cost_map()
        
        # self.tb3_1 = TurtlebotController("tb3_1", self.cost_map)
        # self.tb3_2 = TurtlebotController("tb3_2", self.cost_map)
        
        self.robots = {}
        self.robots["tb3_1"] = {"pose": Pose, "mission": None, "status": STATUS_WAIT, "goal": Pose}
        self.robots["tb3_2"] = self.robots["tb3_1"].copy()
        
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
        
        # self.cube_list[id] = {"status": str, "pose": tuple}
        
        self.header = Header()
        
        self.pose_pub = rospy.Publisher("/poses", PoseStamped, queue_size=10)
        self.mission_pub = rospy.Publisher("/missions", String, queue_size=10) # f"{robot_name} {mission} {area}"
        self.goal_pub = rospy.Publisher("/goals", PoseStamped, queue_size=10)
        
        self.pose_sub = rospy.Subscriber("/tb3/poses", PoseStamped, self.pose_callback, queue_size=10)
        self.status_sub = rospy.Subscriber("/tb3/status", String, self.status_callback ,queue_size=10) # f"{robot_name} {status}"
    
    def pose_callback(self, msg:PoseStamped):
        type, id, area = msg.header.frame_id.split()
        if type == "robot":
            try:
                self.robots[id]["pose"] = msg.pose
            except KeyError:
                pass
        elif type == "cube":
            try:
                self.cube_list[int(id)]["pose"]
            except KeyError:
                self.cube_list[int(id)] = {"status": STATUS_FOUND, "pose": Pose, "area": -1}
            finally:
                self.cube_list[int(id)]["pose"] = msg.pose
                self.cube_list[int(id)]["area"] = int(area)
    
    def status_callback(self, msg: String):
        l = msg.data.split()
        try:
            self.robots[l[0]]["status"] = l[1]
        except KeyError:
            pass
                
    def get_distance(self, robot, target_pos):
        robot_pose:Pose = robot["pose"]
        x = robot_pose.position.x
        y = robot_pose.position.y
        return math.sqrt((x - target_pos[0]) ** 2 + (y - target_pos[1]) ** 2)
    
    def compare_distance(self, area):
        distances = [self.get_distance(self.robots[name], self.area_list[area]["enter_pos"]) for name in self.robots.keys()]
        i = distances.index(min(distances))
        return self.robots.keys()[i]

    def robot_free(self, robot):
        if self.robots[robot] == STATUS_WAIT:
            return True
        else: return False
    
    def duplicate_mission(self, mission, area):
        return any([self.robots[name]["mission"] == (mission, area) for name in self.robots.keys()])
        
    def mission_issue(self, mission, target_id):
        name = ""
        if mission == ACTION_EXPLOR:
            goal_pos = self.area_list[target_id]["enter_pose"]
        else:
            goal_pos = self.cube_list[target_id]["pose"]
        
        if not self.duplicate_mission(mission, target_id):
            if all([self.robot_free(robot) for robot in self.robots.keys()]): 
                robot = self.compare_distance(goal_pos)
                self.robots[robot]["mission"] = (mission, target_id)
                name = robot
            elif self.robot_free("tb3_1"): 
                self.robots["tb3_1"]["mission"] = (mission, target_id)
                name = "tb3_1"
            elif self.robot_free("tb3_2"): 
                self.robots["tb3_2"]["mission"] = (mission, target_id)
                name = "tb3_2"
            else: return False
        else: return False
        
        s = String()
        s.data = f"{name} {mission}"
        self.mission_pub(s)
        
        
        pose = PoseStamped()
        pose.header.frame_id = name
        pose.header.stamp = rospy.Time().now()
        pose.pose.position.x = goal_pos[0]
        pose.pose.position.y = goal_pos[1]
        self.goal_pub.publish(pose)
        
        time.sleep(1)
        return True
    
    def main(self):
        rate = rospy.Rate(1)
        # self.mission_start()
        while not rospy.is_shutdown():
            for robot in self.robots.keys():
                print(f"{robot} status:", self.robots[robot]["pose"], self.robots[robot]["mission"])
            
            # self.tb3_1.other_robot_pose = self.tb3_2.get_pose()
            # self.tb3_2.other_robot_pose = self.tb3_1.get_pose()
            
            if self.area_list[SUBMISSION_AREA]["cube_number"] != 3:
                self.mission_issue(ACTION_EXPLOR, SUBMISSION_AREA)
            else:
                for i in self.area_list[SUBMISSION_AREA]["cube_list"]:
                    try:
                        if self.cube_list[i]["status"] == STATUS_FOUND:
                            mission_issuelished = self.mission_issue(ACTION_PICK_UP, i)
                            if mission_issuelished:
                                self.cube_list[i]["status"] = STATUS_SELECTED
                    except KeyError:
                        continue
            
            for i in range(1, 4):
                if self.area_list[i]["cube_number"] == -1:
                    mission_issuelished = self.mission_issue(ACTION_EXPLOR, i)
                    if mission_issuelished:
                        self.area_list[i]["cube_number"] = 0
                elif len(self.cube_list) != 6 and self.area_list[i]["cube_number"] < 3:
                    self.mission_issue(ACTION_EXPLOR, i)
            
            rate.sleep()
