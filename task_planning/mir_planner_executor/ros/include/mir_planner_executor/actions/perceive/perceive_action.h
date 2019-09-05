/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/perceive/base_perceive_action.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_yb_action_msgs/PerceiveLocationAction.h>

class PerceiveAction : public BasePerceiveAction  {
private:
    actionlib::SimpleActionClient<mir_yb_action_msgs::PerceiveLocationAction> client_;
public:
    PerceiveAction();
protected:
    virtual bool run(std::string& robot, std::string& location);
};
