/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/place/base_place_action.h>

class MockupPlaceAction : public BasePlaceAction  {
protected:
    virtual bool run(std::string& robot, std::string& location, std::string& object);
};
