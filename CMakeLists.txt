cmake_minimum_required(VERSION 2.8.3)
project(resilient_debug_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  landmark_detection
)

catkin_package()
