/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/place/mockup_place_action.h>
#include <mir_planner_executor/actions/mockup_helper.h>

bool MockupPlaceAction::run(std::string& robot, std::string& location, std::string& object) {
    return mockupAsk("place", {robot, location, object});
}
