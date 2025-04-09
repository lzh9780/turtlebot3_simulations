#!/usr/bin/env python

import rospy
import random
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn_random_textured_model():
    # Initialize the ROS node
    rospy.init_node('spawn_random_textured_model', anonymous=True)
    
    # Random the number of target object in three different location, make sure
    # the target object not less than 6. Each location will have 1 to 3 objects. 
    num_list = [random.randint(1, 3) for i in range(3)]
    while sum(num_list) < 6: 
        num_list = [random.randint(1, 3) for i in range(3)]

    # Random target object. The number of target randomed will same as the sum
    # of three positions. Make sure the object will not duplicate. 
    target_list = []
    while len(target_list) in range(sum(num_list)):
        n = random.randint(1, 9)
        if n in target_list:
            pass
        else:
            target_list.append(n)
    
    print(num_list, target_list)
    
    # List of three object ranges. Each range shows as [min_x, min_y, max_x, max_y]
    pose_range_list = [[1.05, -1.95, 1.95, -1.3], [-1.95, -1.95, -1.05, -0.55], [-1.95, 0.55, -1.05, 1.95]]
    
    sdf_path = "~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/cube/model.sdf"

    counter = 0
    for n in target_list:
        try:
            with open(sdf_path, 'r') as f:
                model_xml = f.read()
        except IOError:
            rospy.logerr("Unable to open model SDF file at: %s", sdf_path)
            return
        
        model_xml = model_xml.replace("{mesh_file}", "cube_"+str(n)+".dae")

        if counter < num_list[0]:
            pose_range = pose_range_list[0]
        elif counter < num_list[0] + num_list[1]:
            pose_range = pose_range_list[1]
        else: 
            pose_range = pose_range_list[2]
        
        pose = Pose()
        pose.position = Point(random.uniform(pose_range[0], pose_range[2]), random.uniform(pose_range[1], pose_range[3]), 0.05)
        pose.orientation = Quaternion(0, 0, 0, 1)
        
        rospy.loginfo("Waiting for the /gazebo/spawn_sdf_model service...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            model_name = "random_textured_model" + str(n)
            namespace = ""
            reference_frame = "world"

            resp = spawn_model_client(model_name, model_xml, namespace, pose, reference_frame)
            rospy.loginfo("Spawn status: %s", resp.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            
        counter += 1
    

if __name__ == '__main__':
    try:
        spawn_random_textured_model()
    except rospy.ROSInterruptException:
        pass