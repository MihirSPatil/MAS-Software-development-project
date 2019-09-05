/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/perceive/perceive_action.h>

PerceiveAction::PerceiveAction() : client_("/perceive_location_server") {
    //client_.waitForServer();
}
bool PerceiveAction::run(std::string& robot, std::string& location) {
    mir_yb_action_msgs::PerceiveLocationGoal goal;
    goal.location = location;
    actionlib::SimpleClientGoalState state = client_.sendGoalAndWait(goal, ros::Duration(150.0), ros::Duration(5.0));
    if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
        return false;
    }
    auto res = client_.getResult();
    return res->success;
}
