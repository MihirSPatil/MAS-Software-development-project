/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/unstage/mockup_unstage_action.h>
#include <mir_planner_executor/actions/mockup_helper.h>

bool MockupUnstageAction::run(std::string& robot, std::string& platform, std::string& object) {
    return mockupAsk("unstage", {robot, platform, object});
}
