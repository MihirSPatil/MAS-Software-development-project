cmake_minimum_required(VERSION 2.8.3)
project(mir_align_with_workspace)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    dynamic_reconfigure
    roslint
  )
catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/dynamic_alignment.cfg
)
catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

install(PROGRAMS
  ros/scripts/align_with_workspace_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
