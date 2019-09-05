/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/insert/combined_insert_action.h>
#include <mir_planner_executor/actions/insert/insert_action.h>
#include <mir_planner_executor/actions/insert/insert_cavity_action.h>
#include <utility>
#include <mir_planner_executor/to_be_removed_constants.h>

CombinedInsertAction::CombinedInsertAction() {
    default_insert_ = new InsertAction();
    cavity_insert_ = new InsertCavityAction();
}
void CombinedInsertAction::initialize(KnowledgeUpdater* knowledge_updater) {
    ExecutorAction::initialize(knowledge_updater);
    default_insert_->initialize(knowledge_updater);
    cavity_insert_->initialize(knowledge_updater);
}
bool CombinedInsertAction::run(std::string& name, std::vector<std::string>& arguments) {
    std::string robot = arguments.at(0);
    std::string platform = arguments.at(1);
    std::string location = arguments.at(2);
    std::string peg = arguments.at(3);
    std::string hole = arguments.at(4);
    if(hole == CAVITY_OBJECT_NAME) {
        ROS_INFO("CombinedInsertAction: CAVITY");
        return cavity_insert_->run(name, arguments);
    } else {
        ROS_INFO("CombinedInsertAction: DEFAULT");
        return default_insert_->run(name, arguments);
    }
}
