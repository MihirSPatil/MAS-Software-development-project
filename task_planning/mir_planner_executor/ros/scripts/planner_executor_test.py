#! /usr/bin/env python

import rospy
import actionlib
import os

from mir_plan_loader.msg import LoadPlanAction, LoadPlanGoal
from mir_planner_executor.msg import ExecutePlanAction, ExecutePlanGoal

def loadPlan():
    client = actionlib.SimpleActionClient('load_plan', LoadPlanAction)
    client.wait_for_server()
    goal = LoadPlanGoal()
    goal.file = os.getenv("HOME") + '/.ros/mercury2.plan'
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    plan = client.get_result()
    return plan.plan

def executePlan(plan):
    client = actionlib.SimpleActionClient('execute_plan', ExecutePlanAction)
    client.wait_for_server()
    goal = ExecutePlanGoal()
    goal.plan = plan
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(60.0))
    print client.get_result()

if __name__ == '__main__':
    rospy.init_node('load_plan_client')
    plan = loadPlan()
    executePlan(plan)
