/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/stage/base_stage_action.h>

class MockupStageAction : public BaseStageAction  {
protected:
    virtual bool run(std::string& robot, std::string& platform, std::string& object);
};
