/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/insert/insert_cavity_action.h>

InsertCavityAction::InsertCavityAction() : client_("/insert_object_in_cavity_server") {
    //client_.waitForServer();
}
bool InsertCavityAction::run(std::string& robot, std::string& platform, std::string& location, std::string& peg, std::string& hole) {
    mir_yb_action_msgs::InsertObjectGoal goal;
    goal.peg = peg;
    goal.hole = hole;
    goal.robot_platform = platform;
    actionlib::SimpleClientGoalState state = client_.sendGoalAndWait(goal, ros::Duration(150.0), ros::Duration(5.0));
    if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
        return false;
    }
    auto res = client_.getResult();
    return res->success;
}
