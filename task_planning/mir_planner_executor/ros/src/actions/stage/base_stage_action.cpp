/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/stage/base_stage_action.h>
#include <utility>

void BaseStageAction::update_knowledge_base(bool success, std::string& robot, std::string& platform, std::string& object) {
    if(success) {
        knowledge_updater_->remKnowledge("holding", {{"r", robot}, {"o", object}});

        knowledge_updater_->addKnowledge("gripper_is_free", {{"r", robot}});
        knowledge_updater_->remGoal("gripper_is_free", {{"r", robot}});

        knowledge_updater_->addKnowledge("stored", {{"rp", platform}, {"o", object}});
        knowledge_updater_->remGoal("stored", {{"rp", platform}, {"o", object}});

        knowledge_updater_->addKnowledge("occupied", {{"rp", platform}});
        knowledge_updater_->remGoal("occupied", {{"rp", platform}});
    } else {
        return;
    }
}

bool BaseStageAction::run(std::string& name, std::vector<std::string>& arguments) {
    std::string robot = arguments.at(0);
    std::string platform = arguments.at(1);
    std::string object = arguments.at(2);
    bool success = run(robot, platform, object);
    update_knowledge_base(success, robot, platform, object);
    return success;
}
