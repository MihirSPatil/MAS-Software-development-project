/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/perceive/base_perceive_action.h>

class MockupPerceiveAction : public BasePerceiveAction  {
protected:
    virtual bool run(std::string& robot, std::string& location);
};
