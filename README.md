# TurtleBot3
This repo is changed base on https://github.com/ROBOTIS-GIT/turtlebot3_simulations

## 1 Project Overview
A multi-robot system using two TurtleBot3 platforms (Waffle pi) for collaborative SLAM, navigation, and dynamic ArUco cube tracking. Robots intelligently allocate tracking tasks based on proximity and share environmental awareness to prevent collisions.

## 2 Dependencies
### System
This project based on **Ubuntu 20.04** and **ROS Noetic**. 

For ROS installation, please follow the offical guide. Link: http://wiki.ros.org/noetic/Installation/Ubuntu

### ROS Packages
This Project based on Move Base and AMCL algorithm provided by ROS. They will be installed withthe ROS system. 

### Python Packages
rospy: Will be installed automatically when install ROS in last step. 

OpenCV contrib version: 4.11.0.86

PyQt version: 5.15.11

Numpy version: 2.2.5


## 3 Installation

### Install package
In the terminal run: 
```bash
mkdir -p {your_dir_name}/src
cd {your_dir_name}/src
git clone https://github.com/lzh9780/turtlebot3_simulations.git
cd ..
catkin_make
echo "export PYTHONPATH=$PYTHONPATH:$(pwd)/src/" >> devel/setup.bash
source devel/setup.bash
```
replace {your_dir_name} as your workspace name. 

Optional:
If don't want to source when open a new terminal, run
```bash
echo "source {your_dir_path}/devel/setup.bash" >> ~/.bashrc
```

replace {your_dir_path} as your workspace path. 

### Install python dependency
```bash
pip install numpy opencv-contrib-python pyqt5
```

## 4 Running the system
### 4.1 Simulation
To start the simulator (gazebo), run the command in a new terminal: 
```bash
roslaunch turtlebot3_gazebo turtlebot_playground_multi.launch
```

To start the navigation, run the command in a new terminal: 
```bash
roslaunch turtlebot3_gazebo 430dualnav.launch
```

To start mission, run the command in a new terminal: 
```bash
roslaunch turtlebot3_gazebo multi_robot_controllor.launch
```
After running this command, the robot will begin to execute the mission automatically. 

#### Mapping in Simulator
After playground be replaced, it need to be mapped again. 

**Note**: the start position will be the (0, 0) position of map. 

Use the command to launch a single robot satuation:
```bash
roslaunch turtlebot3_gazebo turtlebot_playground.launch
```

Open a new terminal, run:
```bash
roslaunch turtlebot3_gazebo mapping.launch
```

Install and run keyboard control by usingfollowing command in new terminal:
```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
Key info: 
Move forward: `i`
Move backward: `,`
Rotate left: `j`
Rotate right: `l`
Stop: `k`

To visiualize the map, launch rviz by tapping 
```bash
rviz
```
Then find `Add` in left side bar, Then find `By Topic -> map` to add it into rviz. The laser scan (`By Topic -> LaserScan`) and robot model (`By display type -> RobotModel`) can be added by using the same way. 

To save a map, run command in a new terminal: 
```bash
rosrun map_server map_saver
```
The replace the `map.pgm` and `map.yaml` inside `path_palnning/maps` as new map. 

### 4.2 Start Real test
Make sure you have two turtlebots and a real playground. 

#### Setup your device
Every time open a new terminal, run command:
```bash
export ROS_HOSTNAME={your_ip}
```
or echo it into your `.bashrc` file
```bash
echo "export ROS_HOSTNAME={your_ip}
```

You can use command `ifconfig` to check your ip. Fromat should be like 192.168.0.xxx

#### Bringup robot
**The bringup informatio is foucsing on the turtlebot 02 and 10 in UTS cb11.10 lab. They are modified and different to default robot. Default robot please follow:** https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

Make sure the device and turtlebot are connected to same internet. 

Before connect, the clock for robot need to be reset. 
```bash
ssh ubuntu@192.168.0.2xx "sudo date '$(date +%m%d%H%M%Y.%S)'"
```

Use ssh to connect to robot. 
```bash
ssh ubuntu@192.168.0.2xx
```

Set the ROS_MASTER_URI and ROS_HOSTNAME to ensure it connect to the same roscore on your device. 
```bash
export ROS_MASTER_URI = http://{your_ip}:11311
export ROS_HOSTNAME={turtlebot_ip}
```

After this, bringup the robot by using
```bash
roslaunch turtlebot3_bringup turtlebot3_robot_full.launch
```

#### Mapping
Bring up one of the turtlebot, put it into the real playground. 

**Note**: the start position will be the (0, 0) position of map. 

The method is same as [Mapping in Simulator](), but won't need to start simulator. 

This step is necessary for the navigation. For simulator, the map have been created, but for real world testing, it should be remapped to ensure the location and navigation can run correctly. 

### 4.3 Real World Testing
After bringup two robot, run the following command to publish necessary transform information
```bash
roslaunch turtlebot3_gazebo multi_robot_remote.launch
```

Start navigation
```bash
roslaunch turtlebot3_gazebo 430dualnav.launch
```

To start mission, run the command in a new terminal: 
```bash
roslaunch turtlebot3_gazebo multi_robot_controllor_real.launch
```

## 5. Subsystem specifics

### 5.1 Detection

#### Purpose
This subsystem provide the 'eye' for robot. It could detect the target and object information, then send to the main control for later use. 

#### Key topics/services/files
##### Topics
For simulation: 

Image topic: `{robot_name}/camera/rgb/image_raw`
CameraInfo topic: `{robot_name}/camera/rgb/camera_info`

For real robot: 

Image topic: Depends on the camera on robot
CameraInfo topic: Depends on the camera on robot

Use `rostopic list` to check when robot connected to the host device. 

##### File
This subsystem have been intergrated into the `robot_controllor.py`. See in function `detection()`

##### Configurable settings
`marker_size`: To provide Aruco detector a length of marker for distance calculation. Provide when calling `detection()`

### 5.2 Location and Navigation
#### Purpose
Make the robot know "Where I am" and "Where I need to go to and How can I go to". Using laser scan data and map to get the current location, and planning a path when receive the goal position. Finally, give the robot a command to move. 

#### Key topics/services/files
##### Topics
Velocity topic: `{robot_name}/cmd_vel`

AMCL position topic: `{robot_name}/amcl_pose`

Target goal topic: `{self.robot_name}/move_base/goal`

##### File
The launch file `430dualnav.launch` in turtlebot3_gazebo/launch

All launch file and yaml file in path_planning. 

##### Configurable settings
`first_tb3` and `second_tb3`: Provide the multi robot name for system. Default value is "tb3_1" and "tb3_2"

`first_tb3_x_pos`, `first_tb3_y_pos`, `first_tb3_yaw`: Porvide the start location for first robot. 

`second_tb3_x_pos`, `second_tb3_y_pos`, `second_tb3_yaw`: Porvide the start location for second robot. 

### 5.3 GUI
#### Purpose
Provide a visualization tool to get the running information

#### Key topics/services/files
##### Topics
Image topic: Depends on the camera on robot

Path topic: `/{robot_name}/move_base_node/NavfnROS/plan`

Information topic: `tb3/info` (Self defined topic. type is std_msg String contains a dicrionary. Will be told in [5.4 Intergration and Coopration]())

##### File
Python file `gui.py`, `gui_camera.py`, `gui_map.py` in turtlebot3_gazebo/scripts.  

##### Configurable settings
No configurable settings

### 5.4 Intergration and Coopration
#### Purpose
As the "Brain" of system. To provide mission detail and goal to the robot, and receive the information from robot, include the robot pose, object id and pose, target id and pose. 

#### Key topics/services/files
##### Topics
Information topic: `/info` (Self defined topic. type is std_msg String contains a dicrionary. Data structure:
`{cube_id: {"status": str, "pose": (x, y, z, rx, ry, rz, rw), "area": (area_id:Int)}}`)

Mission topic: `/missions`

Robot information topic: `"/tb3/info"` (Same structure as `/info`)

##### File
Python file `main_control.py`, `main_control_real.py`, `robot_controllor.py`, `robot_controllor_real.py` in turtlebot3_gazebo/scripts. The two file with `_real` is provided for the real world testing, have some difference between the original one (mainly data difference)

`main_control.py` is for mission publish and information storage. 

`robot_controllor.py` is for single robot command execute. For multi robot, it needs to be run multiple times with all robot names. 

##### Configurable settings
`robot_num` for `main_control.py`, default value is 2. The system will wait for the connected robot number equals to input number then start mission. 

`robot_name` for `robot_controllor.py`. define the robot name which same as before. default value is empty string (support single robot launch, but not test)
