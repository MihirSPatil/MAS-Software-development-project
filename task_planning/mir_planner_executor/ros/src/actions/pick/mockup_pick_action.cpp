/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/pick/mockup_pick_action.h>
#include <mir_planner_executor/actions/mockup_helper.h>

bool MockupPickAction::run(std::string& robot, std::string& location, std::string& object) {
    return mockupAsk("pick", {robot, location, object});
}
