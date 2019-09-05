/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/executor_action.h>

class BaseInsertAction : public ExecutorAction  {
private:

public:
    virtual bool run(std::string& name, std::vector<std::string>& arguments);
protected:
    virtual void update_knowledge_base(bool success, std::string& robot, std::string& platform, std::string& location, std::string& peg, std::string& hole);
    virtual bool run(std::string& robot, std::string& platform, std::string& location, std::string& peg, std::string& hole) = 0;
};
