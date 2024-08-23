# Robocon Software Guide

## Table of Contents
  - [Introduction](#introduction)
  - [Important Concepts](#important-concepts)
  - [Tips for Beginners](#tips-for-beginners)
  - [Future Exploration](#future-exploration)
  - [Additional Resources](#additional-resources)
  - [Conclusion](#conclusion)

## Introduction
This guide captures the knowledge acquired while using a depth camera with ROS and Linux during Robocon 2024. It is designed to help beginners get up to speed with the basics of ROS, Linux, depth cameras, and more.

## Roadmap to Learning ROS (Robot Operating System)
1. **Understand the Basics of Linux**
   - **Goal:** Get comfortable with Linux, as ROS primarily runs on it.
   - **Steps:**
     - Learn basic Linux commands (`ls`, `cd`, `sudo`, `apt-get`).
     - Understand file permissions, processes, and networking basics.
     - Practice navigating the filesystem, editing files, and installing packages.

2. **Learn Python or C++**
   - **Goal:** ROS nodes are primarily written in Python or C++.
   - **Steps:**
     - If you're new to programming, start with Python, as it's easier to learn.
     - Learn basic syntax, control flow, data structures, and functions.
     - Practice writing simple scripts and understand how to run them on Linux.

3. **Introduction to ROS**
   - **Goal:** Get familiar with ROS concepts and terminology.
   - **Steps:**
     - Learn what ROS is and why it's used (middleware for robotics).
     - Understand ROS packages, nodes, topics, services, and messages.
     - Install ROS on your Linux system using tutorials from the ROS Wiki.

4. **Work Through ROS Tutorials**
   - **Goal:** Hands-on practice with basic ROS functionality.
   - **Steps:**
     - Follow the beginner tutorials on the ROS Wiki.
     - Learn to create and run simple nodes.
     - Understand how to communicate between nodes using topics.
     - Practice using ROS tools like `rviz` (visualization) and `rqt` (graphical interface).

5. **Explore ROS Packages**
   - **Goal:** Learn to use existing ROS packages to extend functionality.
   - **Steps:**
     - Explore commonly used packages (e.g., `turtlesim` for learning, `navigation` for robot movement).
     - Understand how to find, install, and use packages from the ROS package index.

6. **Create a Simple Robot Simulation**
   - **Goal:** Apply your knowledge to control a simulated robot.
   - **Steps:**
     - Install and learn to use simulation tools like Gazebo.
     - Create a simple robot model and simulate its movement using ROS.
     - Experiment with sensors and actuators in the simulation.

7. **Work on a Real Robot**
   - **Goal:** Transition from simulation to a physical robot.
   - **Steps:**
     - Connect and configure a real robot.
     - Use ROS to control the robot, process sensor data, and perform tasks.
     - Debug and troubleshoot issues that arise in real-world scenarios.

8. **Advanced Topics**
   - **Goal:** Deepen your ROS knowledge for more complex projects.
   - **Steps:**
     - Learn about ROS2 (the next version of ROS) and its differences.
     - Explore advanced ROS concepts like action servers, state machines, and ROS with Docker.
     - Start contributing to open-source ROS projects or develop your own.

9. **Community and Resources**
   - **Goal:** Stay updated and continue learning.
   - **Steps:**
     - Join the ROS community via forums like ROS Answers or the ROS subreddit.
     - Follow ROS-related blogs, webinars, and conferences.
     - Regularly check the ROS Wiki for updates and new tutorials.

## Examples (located in final folder)
1. Training a model to use in object detection 
2. How to manually send messages from terminal prompt to a topic (teleop.py)
3. How to send commands from ROS to arduino (simpleRos2Serial.py)
   1. Python serial and threading lib is used to pass ros msg via serial safely.
   2. Threading's lock ensure only one process can access a shared variable at one time to prevent read or write error in all access it at once.
   3. serial writes to the set serial port under fix baud rate. (both must be same on arduino's side)
4. How to perform object detection using rgbd camera (detect_silo.py)
   1. cvbridge lib is used to convert image format represented as rosmsg to opencv's format for easier process for object detection.
   2. In the callback function of the ros subscriber `imgmsg_to_cv2` is used to convert the image format
   3. Image is then pass into yolov8 model to get result of object detection
   4. The rest is using the result acquire to perform rule based decision making of robot
5. Refer [LOG.md](LOG.md) for more


## Tips
- **Experiment**: Don’t hesitate to try different commands and scripts. Break things to learn how to fix them.
- **Ask Questions**: Use online communities like ROS Answers or Stack Overflow if you get stuck.
- **Document Everything**: Keep a log of what you learn, including mistakes and how you solved them.


## Additional Resources
- **ROS Wiki**: The [ROS Wiki](http://wiki.ros.org/) is your go-to resource for tutorials and documentation.
- **Linux Command Cheat Sheet**: Keep a [Linux command cheat sheet](https://www.geeksforgeeks.org/linux-commands-cheat-sheet/) handy to speed up your workflow.

## Conclusion
This guide is a stepping stone for beginners to build a solid foundation in ROS, Linux, and more. Keep learning, experimenting, and don't be afraid to dive deeper into more advanced topics as you become more comfortable.

### Note
- More complete and working codes are in the `final` folder, others are all the things we experimented with to reach what is in final.