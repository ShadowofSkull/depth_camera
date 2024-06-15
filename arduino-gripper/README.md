Steps to use the custom msgs


1. Update CMakeList 

```

add_message_files(
  FILES
  GripperControl.msg
)

```


Build your dependencies 

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```




