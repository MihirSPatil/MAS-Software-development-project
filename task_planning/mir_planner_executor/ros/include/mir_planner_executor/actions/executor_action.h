/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once
#include <vector>
#include <mir_planner_executor/knowledge_updater.h>

class ExecutorAction {
private:
protected:
    KnowledgeUpdater* knowledge_updater_;
public:
    virtual bool run(std::string& name, std::vector<std::string>& arguments) = 0;
    virtual void initialize(KnowledgeUpdater* knowledge_updater);
};
