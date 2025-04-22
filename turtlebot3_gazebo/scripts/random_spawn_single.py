#!/usr/bin/env python

import rospy
import random
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn_random_textured_model():
    # Initialize the ROS node
    rospy.init_node('spawn_random_textured_model', anonymous=True)
    
    # List of available texture filenames in your model's textures folder
    textures = ['texture1.png', 'texture2.png', 'texture3.png']
    selected_texture = random.choice(textures)
    rospy.loginfo("Randomly selected texture: %s", selected_texture)
    
    # Build the path to your SDF model template
    home_dir = os.path.expanduser("~")
    model_dir = os.path.join(home_dir, ".gazebo", "models", "my_textured_model")
    sdf_path = os.path.join(model_dir, "model.sdf")
    
    try:
        with open(sdf_path, 'r') as f:
            model_xml = f.read()
    except IOError:
        rospy.logerr("Unable to open model SDF file at: %s", sdf_path)
        return

    # Replace the texture placeholder in the SDF with the randomly selected texture.
    # This assumes the SDF uses a placeholder '{texture_file}' where the texture filename should be.
    model_xml = model_xml.replace("{texture_file}", selected_texture)
    
    # Set a random initial pose within a defined area (here between -2 and 2 meters for x and y)
    pose = Pose()
    pose.position = Point(random.uniform(-2, 2), random.uniform(-2, 2), 0)
    pose.orientation = Quaternion(0, 0, 0, 1)

    # Wait for the spawn service to be available
    rospy.loginfo("Waiting for the /gazebo/spawn_sdf_model service...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        model_name = "random_textured_model"  # The name under which it will appear in Gazebo
        namespace = ""  # or you can set a specific namespace for the model
        reference_frame = "world"  # Could also use "gazebo_world" or a link name depending on your use case

        resp = spawn_model_client(model_name, model_xml, namespace, pose, reference_frame)
        rospy.loginfo("Spawn status: %s", resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        spawn_random_textured_model()
    except rospy.ROSInterruptException:
        pass