/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/planner_executor.h>
#include <ros/console.h>

#include <mir_planner_executor/actions/insert/mockup_insert_action.h>
#include <mir_planner_executor/actions/move/mockup_move_action.h>
#include <mir_planner_executor/actions/perceive/mockup_perceive_action.h>
#include <mir_planner_executor/actions/pick/mockup_pick_action.h>
#include <mir_planner_executor/actions/place/mockup_place_action.h>
#include <mir_planner_executor/actions/stage/mockup_stage_action.h>
#include <mir_planner_executor/actions/unstage/mockup_unstage_action.h>

#include <mir_planner_executor/actions/move/move_action.h>
#include <mir_planner_executor/actions/pick/pick_wbc_action.h>
#include <mir_planner_executor/actions/place/place_action.h>
#include <mir_planner_executor/actions/perceive/combined_perceive_action.h>
#include <mir_planner_executor/actions/stage/stage_action.h>
#include <mir_planner_executor/actions/unstage/unstage_action.h>
#include <mir_planner_executor/actions/insert/combined_insert_action.h>

PlannerExecutor::PlannerExecutor(ros::NodeHandle &nh) : server_(nh, "execute_plan", false) {
    server_.registerGoalCallback(boost::bind(&PlannerExecutor::executeCallback, this));
    server_.start();

    ros::NodeHandle private_nh("~");
    private_nh.param<bool>("use_mockups", use_mockups_, false);
    if(use_mockups_) {
        ROS_WARN("PlannerExecutor uses mockups!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    knowledge_updater_ = new KnowledgeUpdater(nh);

    if(use_mockups_) {
        addActionExecutor("PICK", new MockupPickAction());
        addActionExecutor("PLACE", new MockupPlaceAction());
        addActionExecutor("STAGE", new MockupStageAction());
        addActionExecutor("UNSTAGE", new MockupUnstageAction());
        addActionExecutor("MOVE_BASE", new MockupMoveAction());
        addActionExecutor("INSERT", new MockupInsertAction());
        addActionExecutor("PERCEIVE", new MockupPerceiveAction());
    } else {
        addActionExecutor("PICK", new PickWBCAction());
        addActionExecutor("PLACE", new PlaceAction());
        addActionExecutor("STAGE", new StageAction());
        addActionExecutor("UNSTAGE", new UnstageAction());
        addActionExecutor("MOVE_BASE", new MoveAction());
        addActionExecutor("INSERT", new CombinedInsertAction());
        addActionExecutor("PERCEIVE", new CombinedPerceiveAction());
    }
    ROS_INFO("Node initialized. PlannerExecutor is available!!!");
}
PlannerExecutor::~PlannerExecutor() {

}

void PlannerExecutor::addActionExecutor(std::string name, ExecutorAction* action) {
    actions_[toUpper(name)] = action;
    action->initialize(knowledge_updater_);
}
void PlannerExecutor::executeCallback() {
    const mir_planner_executor_msgs::ExecutePlanGoalConstPtr& msg_goal = server_.acceptNewGoal();
    const rosplan_dispatch_msgs::CompletePlan& plan = msg_goal->plan;
    const std::vector<rosplan_dispatch_msgs::ActionDispatch>& actions = plan.plan;
    ROS_INFO("Got plan, %d actions", (unsigned int)actions.size());
    bool result = checkPlan(plan);
    if(!result) {
        ROS_FATAL("Plan check failed. Execution fails");
        mir_planner_executor_msgs::ExecutePlanResult msg_result;
        msg_result.success = false;
        server_.setAborted(msg_result);
        return;
    }
    ROS_INFO("Plan seems to be valid");
    for(auto const& action: actions) {
        if(server_.isPreemptRequested()) {
            ROS_WARN("Preemption is requested. Stopping execution.");
            mir_planner_executor_msgs::ExecutePlanResult msg_result;
            msg_result.success = false;
            server_.setPreempted(msg_result);
            return;
        }
        std::vector<std::string> args = getArguments(action);
        std::string action_name = toUpper(action.name);
        ROS_INFO("Execute action \"%s\"", action_name.c_str());
        ExecutorAction* executor = getActionExecutor(action_name);
        bool res = executor->run(action_name, args);
        if(!res) {
            ROS_WARN("Action \"%s\" failed, abort plan execution", action_name.c_str());
            mir_planner_executor_msgs::ExecutePlanResult msg_result;
            msg_result.success = false;
            server_.setAborted(msg_result);
            return;
        }
        ROS_INFO("Finished action \"%s\"", action_name.c_str());
    }
    ROS_INFO("Plan execution finished");
    mir_planner_executor_msgs::ExecutePlanFeedback msg_feedback;
    mir_planner_executor_msgs::ExecutePlanResult msg_result;
    msg_result.success = true;
    server_.setSucceeded(msg_result);
}

bool PlannerExecutor::checkPlan(const rosplan_dispatch_msgs::CompletePlan& plan) {
    const std::vector<rosplan_dispatch_msgs::ActionDispatch>& actions = plan.plan;
    for(auto const& action: actions) {
        std::string action_name = toUpper(action.name);
        if (!actions_.count(action_name)) {
            ROS_ERROR("Failed to find action executor with name \"%s\" required by the plan.", action_name.c_str());
            return false;
        }
    }
    return true;
}

std::string PlannerExecutor::toUpper(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    return str;
}

ExecutorAction* PlannerExecutor::getActionExecutor(std::string& name) {
    return actions_[toUpper(name)];
}
std::vector<std::string> PlannerExecutor::getArguments(const rosplan_dispatch_msgs::ActionDispatch &action) {
    std::vector<std::string> list;
    for(auto const& p: action.parameters) {
        list.push_back(toUpper(p.value));
    }
    return list;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_executor");

    ros::NodeHandle nh;

    PlannerExecutor planner_executor(nh);

    ros::spin();
    return 0;
}
