cmake_minimum_required(VERSION 2.8.3)
project(mir_plan_loader_msgs)

find_package(catkin REQUIRED
    genmsg
    actionlib_msgs
    actionlib
    rosplan_dispatch_msgs
)

add_action_files(DIRECTORY ros/action FILES LoadPlan.action)
generate_messages(DEPENDENCIES actionlib_msgs rosplan_dispatch_msgs)

catkin_package()
