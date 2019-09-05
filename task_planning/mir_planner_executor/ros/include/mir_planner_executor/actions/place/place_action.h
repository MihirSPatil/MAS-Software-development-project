/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/place/base_place_action.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_yb_action_msgs/PlaceObjectAction.h>

class PlaceAction : public BasePlaceAction  {
private:
    actionlib::SimpleActionClient<mir_yb_action_msgs::PlaceObjectAction> client_;
public:
    PlaceAction();
protected:
    virtual bool run(std::string& robot, std::string& location, std::string& object);
};
