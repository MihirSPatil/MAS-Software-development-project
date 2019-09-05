/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/executor_action.h>
#include <ros/console.h>

void ExecutorAction::initialize(KnowledgeUpdater* knowledge_updater) {
    knowledge_updater_ = knowledge_updater;
}
