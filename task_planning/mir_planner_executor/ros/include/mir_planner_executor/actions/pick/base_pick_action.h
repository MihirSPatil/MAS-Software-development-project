/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/executor_action.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_yb_action_msgs/MoveBaseSafeAction.h>
#include <map>

class BasePickAction : public ExecutorAction  {
private:
    actionlib::SimpleActionClient<mir_yb_action_msgs::MoveBaseSafeAction> move_base_client_;
public:
    BasePickAction();
    virtual bool run(std::string& name, std::vector<std::string>& arguments);
protected:
    virtual void update_knowledge_base(bool success, std::string& robot, std::string& location, std::string& object);
    virtual bool run(std::string& robot, std::string& location, std::string& object) = 0;
    std::map <std::string, int> failure_count_;
};
