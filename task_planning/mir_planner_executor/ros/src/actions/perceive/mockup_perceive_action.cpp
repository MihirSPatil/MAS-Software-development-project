/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/perceive/mockup_perceive_action.h>
#include <mir_planner_executor/actions/mockup_helper.h>

bool MockupPerceiveAction::run(std::string& robot, std::string& location) {
    return mockupAsk("perceive", {robot, location});
}
