/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <ros/ros.h>
#include <map>
#include <vector>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <mir_planner_executor/knowledge_updater.h>
#include <mir_planner_executor_msgs/ExecutePlanAction.h>
#include <mir_planner_executor/actions/executor_action.h>

class PlannerExecutor {
private:
    actionlib::SimpleActionServer<mir_planner_executor_msgs::ExecutePlanAction> server_;
    bool use_mockups_ = false;

    KnowledgeUpdater* knowledge_updater_;

    std::map <std::string, ExecutorAction*> actions_;

    std::vector<std::string> getArguments(const rosplan_dispatch_msgs::ActionDispatch &action);

    ExecutorAction* getActionExecutor(std::string& name);

    bool checkPlan(const rosplan_dispatch_msgs::CompletePlan& plan);
    std::string toUpper(std::string str);
    void addActionExecutor(std::string name, ExecutorAction* action);
public:
    PlannerExecutor(ros::NodeHandle &nh);
    ~PlannerExecutor();
    void executeCallback();
};
