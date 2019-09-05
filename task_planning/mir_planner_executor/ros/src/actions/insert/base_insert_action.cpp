/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/insert/base_insert_action.h>
#include <utility>

void BaseInsertAction::update_knowledge_base(bool success, std::string& robot, std::string& platform, std::string& location, std::string& peg, std::string& hole) {
    if(success) {
        knowledge_updater_->addKnowledge("in", {{"peg", peg}, {"hole", hole}});
        knowledge_updater_->remGoal("in", {{"peg", peg}, {"hole", hole}});

        knowledge_updater_->addKnowledge("on", {{"peg", peg}, {"l", location}});
        knowledge_updater_->remGoal("on", {{"peg", peg}, {"l", location}});

        knowledge_updater_->addKnowledge("heavy", {{"peg", peg}});
        knowledge_updater_->remGoal("heavy", {{"peg", peg}});

        knowledge_updater_->addKnowledge("heavy", {{"hole", hole}});
        knowledge_updater_->remGoal("heavy", {{"hole", hole}});

        knowledge_updater_->remKnowledge("stored", {{"o", peg}, {"rp", platform}});

        knowledge_updater_->remKnowledge("occupied", {{"rp", platform}});
    } else {
        //knowledge_updater_->remKnowledge("perceived", {{"l", location}});
        knowledge_updater_->remGoalsWithObject(peg);
        ROS_WARN("Insert failed, remove goals with object \"%s\"", peg.c_str());
    }
}

bool BaseInsertAction::run(std::string& name, std::vector<std::string>& arguments) {
    std::string robot = arguments.at(0);
    std::string platform = arguments.at(1);
    std::string location = arguments.at(2);
    std::string peg = arguments.at(3);
    std::string hole = arguments.at(4);
    bool success = run(robot, platform, location, peg, hole);
    update_knowledge_base(success, robot, platform, location, peg, hole);
    return success;
}
