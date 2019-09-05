/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/unstage/base_unstage_action.h>

class MockupUnstageAction : public BaseUnstageAction  {
protected:
    virtual bool run(std::string& robot, std::string& platform, std::string& object);
};
