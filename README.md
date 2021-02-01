

Building this repository on the Husky: 
```
catkin_make
source devel/setup.bash
```

Running a specific Python script:
```
rosrun uwb_ros_node pozyx_publisher.py 
```

Steps taken to create this repository structure: 
Create workspace folder which is this repo, create an src directory 
inside of it. Then in that src directory, run
```
catkin_create_pkg uwb_ros_node std_msgs rospy roscpp
```
In the resulting package create scripts folder where 
python scripts are. Modify package CMakeLists.txt file to install python. 


MESSAGE FILE SPECIFICATION
---------------------------------------
Messages that are transmitted by ROS publishers need their format to be specified in 
their own message file in the project. This contains field names as well as datatypes. 
Field names must be be alphanumeric together with underscores, starting with a letter.  
The Python code for generating the header must match the message files. 


Process that was followed for generating headers: 

1) Get the headers by print statement. Create message file as per ROS tutorial. 
2) Replace parantheses with _lpar_, _rpar_
3) Replace spaces with _
4) Replace colons with _colon_
5) Replace (space)0x with zero_x because message has to start with a letter. (space) denotes an actual space. 

FOLLOWING POSSIBLY DEPRECATED, KEPT FOR REFERENCE
---------------------------------------
Requirements installation, in virtual environment. For this project venv is useful
just for PyCharm where it can be selected to see library code from editor. 
Create** virtualenv: 

python3 -m venv pozyx-env

source pozyx-env/bin/activate

pip install -r requirements.txt




