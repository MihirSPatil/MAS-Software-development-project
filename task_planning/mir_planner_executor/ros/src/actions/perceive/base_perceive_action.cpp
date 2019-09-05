/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/perceive/base_perceive_action.h>
#include <utility>

void BasePerceiveAction::update_knowledge_base(bool success, std::string& robot, std::string& location) {
    if(success) {
        knowledge_updater_->addKnowledge("perceived", {{"l", location}});
        knowledge_updater_->remGoal("perceived", {{"l", location}});
    } else {
        //TODO move base?
        return;
    }
}

bool BasePerceiveAction::run(std::string& name, std::vector<std::string>& arguments) {
    std::string robot = arguments.at(0);
    std::string location = arguments.at(1);
    bool success = run(robot, location);
    update_knowledge_base(success, robot, location);
    return success;
}
