/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/insert/base_insert_action.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_yb_action_msgs/InsertObjectAction.h>

class InsertAction : public BaseInsertAction  {
private:
    actionlib::SimpleActionClient<mir_yb_action_msgs::InsertObjectAction> client_;
public:
    InsertAction();
protected:
    virtual bool run(std::string& robot, std::string& platform, std::string& location, std::string& peg, std::string& hole);
};
