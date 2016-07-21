cmake_minimum_required(VERSION 2.8.3)
project(student)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
   FILES
   blob_detect.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  
   geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

