# TurtleBot3
This repo is changed base on https://github.com/ROBOTIS-GIT/turtlebot3_simulations

## Installation
### Install Ros
ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu

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

## Simulation
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

### Mapping in Simulator
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

## Start Real test
Make sure you have two turtlebots and a real playground. 

### Setup your device
Every time open a new terminal, run command:
```bash
export ROS_HOSTNAME={your_ip}
```
or echo it into your `.bashrc` file
```bash
echo "export ROS_HOSTNAME={your_ip}
```

You can use command `ifconfig` to check your ip. Fromat should be like 192.168.0.xxx

### Bringup robot
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

### Mapping
Bring up one of the turtlebot, put it into the real playground. 

**Note**: the start position will be the (0, 0) position of map. 

The method is same as [Mapping in Simulator](###Mapping_in_Simulator), but won't need to start simulator. 

### Test
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
roslaunch turtlebot3_gazebo multi_robot_controllor.launch
```
