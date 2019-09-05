/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/insert/base_insert_action.h>

class MockupInsertAction : public BaseInsertAction  {
protected:
    virtual bool run(std::string& robot, std::string& platform, std::string& location, std::string& peg, std::string& hole);
};
