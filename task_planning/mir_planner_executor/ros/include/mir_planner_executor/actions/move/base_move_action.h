/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/executor_action.h>

class BaseMoveAction : public ExecutorAction  {
private:

public:
    virtual bool run(std::string& name, std::vector<std::string>& arguments);
protected:
    virtual void update_knowledge_base(bool success, std::string& robot, std::string& from, std::string& to);
    virtual bool run(std::string& robot, std::string& from, std::string& to) = 0;
    std::map <std::string, int> failure_count_;
};
