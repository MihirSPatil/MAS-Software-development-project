/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/insert/mockup_insert_action.h>
#include <mir_planner_executor/actions/mockup_helper.h>

bool MockupInsertAction::run(std::string& robot, std::string& platform, std::string& location, std::string& peg, std::string& hole) {
    return mockupAsk("insert", {robot, platform, location, peg, hole});
}
