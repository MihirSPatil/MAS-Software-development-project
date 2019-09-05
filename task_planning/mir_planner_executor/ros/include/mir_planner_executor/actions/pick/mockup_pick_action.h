/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/pick/base_pick_action.h>

class MockupPickAction : public BasePickAction  {
protected:
    virtual bool run(std::string& robot, std::string& location, std::string& object);
};
