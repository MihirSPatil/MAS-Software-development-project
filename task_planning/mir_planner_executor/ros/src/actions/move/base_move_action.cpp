/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/move/base_move_action.h>
#include <utility>

void BaseMoveAction::update_knowledge_base(bool success, std::string& robot, std::string& from, std::string& to) {
    int N = 0;
    if(success) {
        knowledge_updater_->remKnowledge("at", {{"r", robot}, {"l", from}});
        knowledge_updater_->remGoal("at", {{"r", robot}, {"l", to}});
        knowledge_updater_->addKnowledge("at", {{"r", robot}, {"l", to}});
        knowledge_updater_->remKnowledge("perceived", {{"l", from}});
        knowledge_updater_->remKnowledge("perceived", {{"l", to}});
    } else {
        int count = 1;
        if(failure_count_.find(to) != failure_count_.end()) {
            count = failure_count_[to] + 1;
        }
        ROS_WARN("Move to location \"%s\" failed %d times", to.c_str(), count);
        if(count > N) {
            knowledge_updater_->remGoalsRelatedToLocation(to);
            ROS_WARN("Move failed %d times, remove goals with object from location \"%s\"", count, to.c_str());
            failure_count_[to] = 0;
        } else {
            failure_count_[to] = count;
        }
    }
}

bool BaseMoveAction::run(std::string& name, std::vector<std::string>& arguments) {
    std::string robot = arguments.at(0);
    std::string from = arguments.at(1);
    std::string to = arguments.at(2);
    bool success = run(robot, from, to);
    update_knowledge_base(success, robot, from, to);
    return success;
}
