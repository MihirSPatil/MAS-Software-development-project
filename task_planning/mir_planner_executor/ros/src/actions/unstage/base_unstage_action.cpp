/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/unstage/base_unstage_action.h>
#include <utility>

void BaseUnstageAction::update_knowledge_base(bool success, std::string& robot, std::string& platform, std::string& object) {
    if(success) {
        knowledge_updater_->remKnowledge("gripper_is_free", {{"r", robot}});
        knowledge_updater_->remKnowledge("stored", {{"rp", platform}, {"o", object}});
        knowledge_updater_->remKnowledge("occupied", {{"rp", platform}});

        knowledge_updater_->addKnowledge("holding", {{"r", robot}, {"o", object}});
        knowledge_updater_->remGoal("holding", {{"r", robot}, {"o", object}});
    } else {
        return;
    }
}

bool BaseUnstageAction::run(std::string& name, std::vector<std::string>& arguments) {
    std::string robot = arguments.at(0);
    std::string platform = arguments.at(1);
    std::string object = arguments.at(2);
    bool success = run(robot, platform, object);
    update_knowledge_base(success, robot, platform, object);
    return success;
}
