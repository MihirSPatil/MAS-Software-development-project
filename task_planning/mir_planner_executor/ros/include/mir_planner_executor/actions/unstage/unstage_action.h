/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/unstage/base_unstage_action.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_yb_action_msgs/UnStageObjectAction.h>

class UnstageAction : public BaseUnstageAction  {
private:
    actionlib::SimpleActionClient<mir_yb_action_msgs::UnStageObjectAction> client_;
public:
    UnstageAction();
protected:
    virtual bool run(std::string& robot, std::string& platform, std::string& object);
};
