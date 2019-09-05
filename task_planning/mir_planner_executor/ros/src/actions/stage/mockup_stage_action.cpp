/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/stage/mockup_stage_action.h>
#include <mir_planner_executor/actions/mockup_helper.h>

bool MockupStageAction::run(std::string& robot, std::string& platform, std::string& object) {
    return mockupAsk("stage", {robot, platform, object});
}
