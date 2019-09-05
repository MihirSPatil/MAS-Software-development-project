/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/stage/stage_action.h>

StageAction::StageAction() : client_("/stage_object_server") {
    //client_.waitForServer();
}
bool StageAction::run(std::string& robot, std::string& platform, std::string& object) {
    mir_yb_action_msgs::StageObjectGoal goal;
    goal.robot_platform = platform;
    actionlib::SimpleClientGoalState state = client_.sendGoalAndWait(goal, ros::Duration(150.0), ros::Duration(5.0));
    if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
        return false;
    }
    auto res = client_.getResult();
    return res->success;
}
