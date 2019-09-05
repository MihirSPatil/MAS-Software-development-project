/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/pick/base_pick_action.h>
#include <utility>

BasePickAction::BasePickAction() : move_base_client_("/move_base_safe_server") {
    //move_base_client_.waitForServer();
}

void BasePickAction::update_knowledge_base(bool success, std::string& robot, std::string& location, std::string& object) {
    int N = 1;
    if(success) {
        knowledge_updater_->addKnowledge("holding", {{"r", robot}, {"o", object}});
        knowledge_updater_->remGoal("holding", {{"r", robot}, {"o", object}});
        knowledge_updater_->remKnowledge("on", {{"o", object}, {"l", location}});
        knowledge_updater_->remKnowledge("gripper_is_free", {{"r", robot}});
    } else {
        int count = 1;
        if(failure_count_.find(object) != failure_count_.end()) {
            count = failure_count_[object] + 1;
        }
        ROS_WARN("Pick for object \"%s\" failed %d times", object.c_str(), count);
        if(count > N) {
            knowledge_updater_->remGoalsWithObject(object);
            ROS_WARN("Pick failed %d times, remove goals with object \"%s\"", count, object.c_str());
            failure_count_[object] = 0;
        } else {
            knowledge_updater_->remKnowledge("perceived", {{"l", location}});
            failure_count_[object] = count;
        }

        mir_yb_action_msgs::MoveBaseSafeGoal goal;
        goal.arm_safe_position = "folded";
        goal.source_location = location;
        goal.destination_location = location;
        goal.dont_be_safe = true;
        move_base_client_.sendGoalAndWait(goal, ros::Duration(5.0), ros::Duration(5.0));

        return;
    }
}

bool BasePickAction::run(std::string& name, std::vector<std::string>& arguments) {
    std::string robot = arguments.at(0);
    std::string location = arguments.at(1);
    std::string object = arguments.at(2);
    bool success = run(robot, location, object);
    update_knowledge_base(success, robot, location, object);
    return success;
}
