#!/usr/bin/env python

import argparse
from turtlebot3_simulations.turtlebot3_gazebo.scripts.status import *
import math
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time
from copy import deepcopy
import json

class MainControl:
    def __init__(self, robot_num=2):
        # Store robot information 
        self.robot_num = robot_num
        self.robots = {}
        self.area_list = {}     # {area_id: {"enter_pos": (x, y, z, rx, ry, rz, rw), "cube_numer": Int}}
        self.cube_list = {}     # {cube_id: {"status": str, "pose": (x, y, z, rx, ry, rz, rw), "area": (area_id:Int)}}
        self.markers = {}       # {marker_id: {"current_id": Int, "pose": (x, y, z, rx, ry, rz, rw), "status": str}}
        
        # self.robots["tb3_1"] = {"pose": None, "mission": None, "status": STATUS_WAIT, "goal": None}
        # self.robots["tb3_2"] = deepcopy(self.robots["tb3_1"])
        
        self.area_list[SUBMISSION_AREA] = {"enter_pos": (-0.5, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0), "cube_number":-1}
        self.area_list[STORE_AREA_1] = {"enter_pos": (0.0, -0.25, 0.0, 0.0, 0.0, -0.9239, 0.3827), "cube_number":-1}
        self.area_list[STORE_AREA_2] = {"enter_pos": (-1.25, -0.7, 0.0, 0.0, 0.0, 0.0, 1.0), "cube_number":-1}
        self.area_list[STORE_AREA_3] = {"enter_pos": (1.25, -0.25, 0.0, 0.0, 0.0, 0.0, 1.0), "cube_number":-1}
        
        self.mission_pub = rospy.Publisher("/missions", String, queue_size=10) # 
        
        self.info_sub = rospy.Subscriber("/tb3/info", String, self.info_callback, queue_size=10)
    
    def info_callback(self, msg:String):
        # Input string: {"type": str, "id": str, "info": Dict}
        d = json.loads(msg.data)
        
        try:
            try:
                if d["info"]["pose"] != None:
                    x, y, z, rx, ry, rz, rw = d["info"]["pose"]
            except (ValueError, TypeError):
                print("Invaild pose: ", d["info"]["pose"])
                print("message: ", d)
            
            i = d["id"]
            if d["type"] == "robot":
                try:
                    self.robots[i]
                except KeyError as e:
                    self.robots[i] = {"pose": None, "mission": None, "status": None, "goal": None}
                    print(f"New robot connected: {i}")
                finally:
                    self.robots[i]["pose"] = d["info"]["pose"]
                    if self.robots[i]["status"] == STATUS_MISSION_PUB and d["info"]["mission"] == None:
                        return
                    self.robots[i]["status"] = d['info']["status"]
                    self.robots[i]["mission"] = d["info"]["mission"]

            elif d["type"] == "cube":
                try:
                    self.cube_list[int(i)]
                except KeyError:
                    self.cube_list[int(i)] = {"status": STATUS_FOUND, "pose": None, "area": int(d["info"]["area"])}
                    self.area_list[int(d["info"]["area"])]["cube_number"] += 1
                    print(f"New cube found: {i}")
                finally:
                    if d["info"]["pose"] != None: 
                        self.cube_list[int(i)]["pose"] = d["info"]["pose"]
                        self.cube_list[int(i)]["area"] = int(d["info"]["area"])
                    if d["info"]["status"] == STATUS_SUBMITTED and self.cube_list[int(i)]["status"] != STATUS_SUBMITTED:
                        print(f"Cube {i} submitted")
                    self.cube_list[int(i)]["status"] = d["info"]["status"]
            
            elif d["type"] == "marker":
                try:
                    self.markers[int(i)]
                except KeyError:
                    self.markers[int(i)] = {"pose": None, "current_id": int(i), "status": STATUS_FOUND}
                    print(f"New target found: {i}")
                finally:
                    self.markers[int(i)]["pose"] = d["info"]["pose"]
                    self.markers[int(i)]["status"] = d["info"]["status"]
        except KeyError:
            print("Invalid message: ", msg.data)
                
    def get_distance(self, robot, target_pos):
        robot_pose:Pose = robot["pose"]
        x = robot_pose[0]
        y = robot_pose[1]
        return math.sqrt((x - target_pos[0]) ** 2 + (y - target_pos[1]) ** 2)
    
    def compare_distance(self, area):
        distances = [self.get_distance(self.robots[name], area) for name in self.robots.keys()]
        i = distances.index(min(distances))
        j = 0
        for robot in self.robots.keys():
            if i == j:
                return robot
            else:
                j += 1

    def robot_free(self, robot):
        if self.robots[robot]["status"] == STATUS_WAIT and self.robots[robot]["mission"] == None:
            return True
        else: return False
    
    def duplicate_mission(self, mission, area):
        return any([self.robots[name]["mission"] == [mission, area] for name in self.robots.keys()])
        
    def mission_issue(self, mission, target_id):
        name = ""
        if mission == ACTION_EXPLOR:
            goal_pose = self.area_list[target_id]["enter_pos"]
        elif mission == ACTION_PICK_UP:
            goal_pose = self.cube_list[target_id]["pose"]
            goal_pose[5] = self.area_list[self.cube_list[target_id]["area"]]["enter_pos"][5]
            goal_pose[6] = self.area_list[self.cube_list[target_id]["area"]]["enter_pos"][6]
        elif mission == ACTION_SUBMIT:
            goal_pose = self.markers[target_id]["pose"]
            goal_pose[5] = 1.0
            goal_pose[6] = 0.0
        
        if mission != ACTION_SUBMIT and not self.duplicate_mission(mission, target_id):
            if all([self.robot_free(robot) for robot in self.robots.keys()]): 
                name = self.compare_distance(goal_pose)
            elif self.robot_free("tb3_1"): 
                name = "tb3_1"
            elif self.robot_free("tb3_2"): 
                name = "tb3_2"
            else: return False
        elif mission == ACTION_SUBMIT:
            for robot in self.robots.keys():
                if self.robots[robot]["mission"] == [ACTION_SUBMIT, target_id]:
                    name = robot
                    break
        else: return False
        
        if mission != ACTION_SUBMIT:
            self.robots[name]["mission"] = (mission, target_id)
            self.robots[name]["status"] = STATUS_MISSION_PUB
        
            s = String()
            s.data = json.dumps({"id": name, "mission": (mission, target_id), "goal": goal_pose})
            self.mission_pub.publish(s)
        else: 
            s = String()
            s.data = json.dumps({"id": name, "mission": (mission, target_id), "goal": self.markers[target_id]["pose"]})
            self.mission_pub.publish(s)
        
        time.sleep(1)
        return True
    
    def main(self):
        rate = rospy.Rate(1)
        # self.mission_start()
        while not rospy.is_shutdown():
            rate.sleep()
            
            if len(self.robots) < self.robot_num:
                print(f"No enough robot to start. Current robot: {len(self.robots)}, expect: {self.robot_num}")
                continue
            
            # for robot in self.robots.keys():
            #     m = self.robots[robot]["mission"]
            #     s = self.robots[robot]["status"]
            #     print(f"{robot} mission: {m} status: {s}")

            if any([self.robot_free(robot) or self.robots[robot]["mission"][0] == ACTION_SUBMIT for robot in self.robots.keys()]):
                if len(self.markers) != 3:
                    self.mission_issue(ACTION_EXPLOR, SUBMISSION_AREA)

                if len(self.cube_list) != 6:
                    for i in range(1, 4):
                        if self.area_list[i]["cube_number"] == -1:
                            mission_published = self.mission_issue(ACTION_EXPLOR, i)
                            if mission_published:
                                self.area_list[i]["cube_number"] = 0
                
                    if all([self.area_list[i]["cube_number"] >= 0 for i in range(1, 4)]) and self.area_list[i]["cube_number"] < 3:
                        self.mission_issue(ACTION_EXPLOR, i)

                if len(self.markers) > 0:
                    for i in self.markers.keys():
                        try:
                            if self.cube_list[i]["status"] == STATUS_FOUND:
                                mission_published = self.mission_issue(ACTION_PICK_UP, i)
                                if mission_published:
                                    self.cube_list[i]["status"] = STATUS_SELECTED
                            elif self.cube_list[i]["status"] == STATUS_PICKED:
                                print(f"Submitting cube {i}")
                                self.mission_issue(ACTION_SUBMIT, i)
                        except KeyError:
                            continue
            
            # for i in range(1, 4):
            #     if self.area_list[i]["cube_number"] == -1:
            #         mission_published = self.mission_issue(ACTION_EXPLOR, i)
            #         if mission_published:
            #             self.area_list[i]["cube_number"] = 0
            
            #     if all([self.area_list[i]["cube_number"] >= 0 for i in range(1, 4)]) and len(self.cube_list) != 6 and self.area_list[i]["cube_number"] < 2:
            #         self.mission_issue(ACTION_EXPLOR, i)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Arguments")
    parser.add_argument("robot_num", default=2)
    args, unknown = parser.parse_known_args()
    
    rospy.init_node("main_control")
    control = MainControl(int(args.robot_num))
    control.main()
