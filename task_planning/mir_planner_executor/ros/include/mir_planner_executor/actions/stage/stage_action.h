/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/stage/base_stage_action.h>
#include <actionlib/client/simple_action_client.h>
#include <mir_yb_action_msgs/StageObjectAction.h>

class StageAction : public BaseStageAction  {
private:
    actionlib::SimpleActionClient<mir_yb_action_msgs::StageObjectAction> client_;
public:
    StageAction();
protected:
    virtual bool run(std::string& robot, std::string& platform, std::string& object);
};
