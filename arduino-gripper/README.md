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

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
```


Update package.xml, with message_generation and message_runtime 

```
<?xml version="1.0"?>
<package format="2">

  #HERE 
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  #END
  <exec_depend>message_runtime</exec_depend>
  <build_depend>rospy</build_depend>
  <exec_depend>rospy</exec_depend>
  <build_depend>std_msgs</build_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```

Build your dependencies 

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```




