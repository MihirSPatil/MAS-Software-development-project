/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/executor_action.h>

class CombinedPerceiveAction : public ExecutorAction  {
private:
    ExecutorAction* default_perceive_;
    ExecutorAction* cavity_perceive_;
public:
    CombinedPerceiveAction();
    virtual bool run(std::string& name, std::vector<std::string>& arguments);
    virtual void initialize(KnowledgeUpdater* knowledge_updater);
};
