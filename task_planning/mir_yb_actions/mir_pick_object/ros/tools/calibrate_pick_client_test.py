#! /usr/bin/env python
import rospy
import roslib
import actionlib

import sys

from mir_yb_action_msgs.msg import PickObjectWBCAction, PickObjectWBCGoal

if __name__ == '__main__':
    rospy.init_node('calibrate_pick_client_test')
    client = actionlib.SimpleActionClient('calibrate_pick_server', PickObjectWBCAction)
    rospy.loginfo(str(len(sys.argv)))
    client.wait_for_server()
    goal = PickObjectWBCGoal()

    if len(sys.argv) < 2:
        print ('Required parameters for moving arm ')
        print ('Usage : ./calibrate_pick_test.py <arm_location>')
        exit()

    try:
        goal.object = sys.argv[1]
        timeout = 150.0
        rospy.loginfo("sending goal")
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        print client.get_result()
    except:
        rospy.loginfo("exception")
        pass
