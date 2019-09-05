#! /usr/bin/env python

import rospy
import actionlib
import os

from mir_plan_loader.msg import LoadPlanAction, LoadPlanGoal

if __name__ == '__main__':
    rospy.init_node('load_plan_client')
    client = actionlib.SimpleActionClient('load_plan', LoadPlanAction)
    client.wait_for_server()
    goal = LoadPlanGoal()
    goal.file = os.getenv("HOME") + '/.ros/mercury.plan'
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    result = client.get_result()
    print result
