/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/pick/base_pick_action.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_yb_action_msgs/PickObjectWBCAction.h>

class PickWBCAction : public BasePickAction  {
private:
    actionlib::SimpleActionClient<mir_yb_action_msgs::PickObjectWBCAction> client_;
public:
    PickWBCAction();
protected:
    virtual bool run(std::string& robot, std::string& location, std::string& object);
};
