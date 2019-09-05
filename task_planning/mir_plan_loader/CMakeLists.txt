cmake_minimum_required(VERSION 2.8.3)
project(mir_plan_loader)

find_package(catkin REQUIRED
  COMPONENTS
  genmsg
  actionlib_msgs
  actionlib
  rosplan_dispatch_msgs
  mir_plan_loader_msgs
)


catkin_package(
  CATKIN_DEPENDS
)
