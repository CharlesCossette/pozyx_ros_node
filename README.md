

Building this repository on the Husky: 
```
catkin_make
source devel/setup.bash
```

Running a specific Python script:
```
rosrun pozyx_ros_node pozyx_publisher.py 
```

Steps taken to create this repository structure: 
Create workspace folder which is this repo, create an src directory 
inside of it. Then in that src directory, run
```
catkin_create_pkg pozyx_ros_node std_msgs rospy roscpp
```
In the resulting package create scripts folder where 
python scripts are. Modify package CMakeLists.txt file to install python. 

rosbag record /gx5/filtered/imu/data /gx5/imu/data /pozyx_imu /pozyx_range



MESSAGE FILE SPECIFICATION
---------------------------------------
Messages that are transmitted by ROS publishers need their format to be specified in 
their own message file in the project. This contains field names as well as datatypes. 
Field names must be be alphanumeric together with underscores, starting with a letter.  
The Python code for generating the header must match the message files. 


FOLLOWING POSSIBLY DEPRECATED, KEPT FOR REFERENCE
---------------------------------------
Requirements installation, in virtual environment. For this project venv is useful
just for PyCharm where it can be selected to see library code from editor. 
Create** virtualenv: 

python3 -m venv pozyx-env

source pozyx-env/bin/activate

pip install -r requirements.txt




