/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/move/base_move_action.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_yb_action_msgs/MoveBaseSafeAction.h>

class MoveAction : public BaseMoveAction  {
private:
    actionlib::SimpleActionClient<mir_yb_action_msgs::MoveBaseSafeAction> client_;
public:
    MoveAction();
protected:
    virtual bool run(std::string& robot, std::string& from, std::string& to);
};
