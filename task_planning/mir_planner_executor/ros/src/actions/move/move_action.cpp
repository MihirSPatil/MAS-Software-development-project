/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/move/move_action.h>

MoveAction::MoveAction() : client_("/move_base_safe_server") {
    //client_.waitForServer();
}
bool MoveAction::run(std::string& robot, std::string& from, std::string& to) {
    mir_yb_action_msgs::MoveBaseSafeGoal goal;
    goal.arm_safe_position = "folded";
    goal.source_location = from;
    goal.destination_location = to;
    actionlib::SimpleClientGoalState state = client_.sendGoalAndWait(goal, ros::Duration(150.0), ros::Duration(5.0));
    if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
        return false;
    }
    auto res = client_.getResult();
    return res->success;
}
