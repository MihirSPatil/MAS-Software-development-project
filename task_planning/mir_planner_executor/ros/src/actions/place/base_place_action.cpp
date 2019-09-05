/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/place/base_place_action.h>
#include <utility>

void BasePlaceAction::update_knowledge_base(bool success, std::string& robot, std::string& location, std::string& object) {
    if(success) {
        knowledge_updater_->addKnowledge("on", {{"o", object}, {"l", location}});
        knowledge_updater_->remGoal("on", {{"o", object}, {"l", location}});
        knowledge_updater_->remKnowledge("holding", {{"r", robot}, {"o", object}});
        knowledge_updater_->addKnowledge("gripper_is_free", {{"r", robot}});
        knowledge_updater_->remGoal("gripper_is_free", {{"r", robot}});
        knowledge_updater_->addKnowledge("perceived", {{"l", location}});
        knowledge_updater_->remGoal("perceived", {{"l", location}});
    } else {
        return;
    }
}

bool BasePlaceAction::run(std::string& name, std::vector<std::string>& arguments) {
    std::string robot = arguments.at(0);
    std::string location = arguments.at(1);
    std::string object = arguments.at(2);
    bool success = run(robot, location, object);
    update_knowledge_base(success, robot, location, object);
    return success;
}
