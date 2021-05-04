# Pozyx ROS Node
This repository contains functionality to connect to a Pozyx developer tag over USB, and publish range measurements to nearby devices on ROS topics. Pozyx devices within UWB range are automatically discovered, and multiple Pozyx devices connected to the local computer by USB is also supported. 

## Setup

First, clone this repository inside the `src` folder of your catkin workspace.
```
cd <YOUR_WORKSPACE_FOLDER>/src
git clone https://bitbucket.org/decargroup/pozyx_ros_node.
```
> Note: Using the url above corresponds to an HTTPS login, which will require you to type in your Bitbucket username and password every time you connect to the remote repo. Feel free to also use SSH-keys for this step.

Next, still in the top-level catkin workspace folder, build the workspace using `catkin_make`, source the setup script, and install this package's dependencies.
```
cd <YOUR_WORKSPACE_FOLDER>
catkin_make
source ./devel/setup.bash
rosdep install pozyx_ros_node
```
> Note: you can also use the newer `catkin build` instead of `catkin_make`, but you need to stick to one or the other.

Then, launch the node with 
```
rosrun pozyx_ros_node pozyx_node.launch. 
```
Alternatively, you can start the individual node randomly, after having started a ROS Core.
```
rosrun pozyx_ros_node pozyx_node.py 
```
Range/distance data is available on the `/pozyx_range` topic, and the imu data is available on the  `/pozyx_range`. To see the output you can open a terminal and type
```
rostopic echo /pozyx_range
```

## TODO
1. Positioning still needs to be implemented.
2. Check serial connection issue on the husky. Sometimes the husky wont automatically detect a Pozyx device is connected by USB.
3. allow_self_ranging to be a user option.
4. finish implementation for multiple pozyx devices.



