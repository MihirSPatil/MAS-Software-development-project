/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/move/base_move_action.h>

class MockupMoveAction : public BaseMoveAction  {
protected:
    virtual bool run(std::string& robot, std::string& from, std::string& to);
};
