cmake_minimum_required(VERSION 2.8.3)
project(mir_planner_executor_msgs)

find_package(catkin REQUIRED
    genmsg
    actionlib_msgs
    actionlib
    rosplan_dispatch_msgs
    message_generation
)

add_action_files(DIRECTORY ros/action FILES ExecutePlan.action)
add_service_files(DIRECTORY ros/srv FILES ReAddGoals.srv)
generate_messages(DEPENDENCIES actionlib_msgs rosplan_dispatch_msgs)
catkin_package()
