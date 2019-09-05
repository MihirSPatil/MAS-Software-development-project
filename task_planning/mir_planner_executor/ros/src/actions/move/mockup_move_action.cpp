/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/move/mockup_move_action.h>
#include <mir_planner_executor/actions/mockup_helper.h>

bool MockupMoveAction::run(std::string& robot, std::string& from, std::string& to) {
    return mockupAsk("move_base", {robot, from, to});
}
