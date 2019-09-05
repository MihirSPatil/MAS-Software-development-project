/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/place/place_action.h>

PlaceAction::PlaceAction() : client_("/place_object_server") {
    //client_.waitForServer();
}
bool PlaceAction::run(std::string& robot, std::string& location, std::string& object) {
    mir_yb_action_msgs::PlaceObjectGoal goal;
    goal.object = object;
    goal.location = location;
    actionlib::SimpleClientGoalState state = client_.sendGoalAndWait(goal, ros::Duration(150.0), ros::Duration(5.0));
    if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
        return false;
    }
    auto res = client_.getResult();
    return res->success;
}
