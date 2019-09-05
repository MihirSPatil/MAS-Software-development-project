cmake_minimum_required(VERSION 2.8.3)
project(mir_planner_executor)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslint
    rosplan_planning_system
    std_msgs
    genmsg
    actionlib_msgs
    actionlib
    rosplan_dispatch_msgs
    mir_yb_action_msgs
    mir_planner_executor_msgs
)

add_definitions(-std=c++11)
catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
)


### EXECUTABLES
add_executable(planner_executor
    ros/src/planner_executor.cpp
    ros/src/knowledge_updater.cpp
    ros/src/actions/executor_action.cpp

    ros/src/actions/mockup_helper.cpp
    ros/src/actions/move/mockup_move_action.cpp
    ros/src/actions/insert/mockup_insert_action.cpp
    ros/src/actions/perceive/mockup_perceive_action.cpp
    ros/src/actions/pick/mockup_pick_action.cpp
    ros/src/actions/place/mockup_place_action.cpp
    ros/src/actions/stage/mockup_stage_action.cpp
    ros/src/actions/unstage/mockup_unstage_action.cpp

    ros/src/actions/move/base_move_action.cpp
    ros/src/actions/move/move_action.cpp
    ros/src/actions/pick/base_pick_action.cpp
    ros/src/actions/pick/pick_wbc_action.cpp
    ros/src/actions/place/base_place_action.cpp
    ros/src/actions/place/place_action.cpp
    ros/src/actions/perceive/base_perceive_action.cpp
    ros/src/actions/perceive/perceive_action.cpp
    ros/src/actions/perceive/perceive_cavity_action.cpp
    ros/src/actions/perceive/combined_perceive_action.cpp
    ros/src/actions/stage/base_stage_action.cpp
    ros/src/actions/stage/stage_action.cpp
    ros/src/actions/unstage/base_unstage_action.cpp
    ros/src/actions/unstage/unstage_action.cpp
    ros/src/actions/insert/base_insert_action.cpp
    ros/src/actions/insert/insert_action.cpp
    ros/src/actions/insert/insert_cavity_action.cpp
    ros/src/actions/insert/combined_insert_action.cpp
)
target_link_libraries(planner_executor ${catkin_LIBRARIES})

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

roslint_cpp()

### INSTALLS

install(DIRECTORY ros/include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(TARGETS planner_executor
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
