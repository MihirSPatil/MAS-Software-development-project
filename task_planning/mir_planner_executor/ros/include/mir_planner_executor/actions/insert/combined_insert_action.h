/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/executor_action.h>

class CombinedInsertAction : public ExecutorAction  {
private:
    ExecutorAction* default_insert_;
    ExecutorAction* cavity_insert_;
public:
    CombinedInsertAction();
    virtual bool run(std::string& name, std::vector<std::string>& arguments);
    virtual void initialize(KnowledgeUpdater* knowledge_updater);
};
