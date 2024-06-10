Steps to use the custom msgs


1. Update CMakeList 

```
cmake_minimum_required(VERSION 3.0.2)
project(gripper_control_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GripperControl.msg
)

```


Update package.xml, with message_generation and message_runtime 

```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>

```

Build your dependencies 

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```




