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

## Usage

Launch the node with 
```
roslaunch pozyx_ros_node pozyx_node.launch. 
```
Alternatively, you can start the individual node randomly, after having started a ROS Core.
```
rosrun pozyx_ros_node pozyx_node.py 
```
Range/distance data is available on the `/pozyx/DEVICE_ID/range` topic, and the imu data is available on the  `/pozyx/DEVICE_ID/imu`. To see the output you can open a terminal and type
```
rostopic echo /pozyx/DEVICE_ID/range
```
# Specifying UWB Anchor Coordinates (for positioning)
To get the UWB modules to actually calculate x, y, z position coordinates, the anchor positions need to be specified. This can be found in the `./config/anchors.yaml` folder. Specify the anchors accordingly. The YAML file is loaded when the `pozyx_node` is initalized. Hence, if you make changes to `anchors.yaml`, you need to restart/relaunch the node.

## TODO
1. Positioning needs to be finalized. Its all implemented, its just that it needs to be turned on with a user option, not on by default, since it wont work if there arnt enough anchors in the room.
2. allow more user options for all sorts of data collection settings.



