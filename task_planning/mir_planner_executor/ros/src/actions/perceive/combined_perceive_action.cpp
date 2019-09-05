/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/perceive/combined_perceive_action.h>
#include <mir_planner_executor/actions/perceive/perceive_action.h>
#include <mir_planner_executor/actions/perceive/perceive_cavity_action.h>
#include <utility>
#include <mir_planner_executor/to_be_removed_constants.h>

CombinedPerceiveAction::CombinedPerceiveAction() {
    default_perceive_ = new PerceiveAction();
    cavity_perceive_ = new PerceiveCavityAction();
}
void CombinedPerceiveAction::initialize(KnowledgeUpdater* knowledge_updater) {
    ExecutorAction::initialize(knowledge_updater);
    default_perceive_->initialize(knowledge_updater);
    cavity_perceive_->initialize(knowledge_updater);
}
bool CombinedPerceiveAction::run(std::string& name, std::vector<std::string>& arguments) {
    std::string robot = arguments.at(0);
    std::string location = arguments.at(1);
    if(location == CAVITY_LOCATION_NAME) {
        ROS_INFO("CombinedPerceiveAction: CAVITY");
        return cavity_perceive_->run(name, arguments);
    } else {
        ROS_INFO("CombinedPerceiveAction: DEFAULT");
        return default_perceive_->run(name, arguments);
    }
}
