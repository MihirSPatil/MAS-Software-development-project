#! /usr/bin/env python
import rospy
import roslib
import actionlib

import sys

from mir_yb_action_msgs.msg import InsertObjectAction, InsertObjectGoal

if __name__ == '__main__':
    rospy.init_node('insert_object_in_cavity_client_tester')
    client = actionlib.SimpleActionClient('insert_object_in_cavity_server', InsertObjectAction)
    client.wait_for_server()
    goal = InsertObjectGoal()
    print "found server"
    if len(sys.argv) == 2: # 1 arguments were received : ok proceed
        try:
            goal.hole = "PP01"
            goal.peg = str(sys.argv[1])
            goal.robot_platform = 'platform_middle'
            timeout = 15.0
            rospy.loginfo('Sending action lib goal to insert_object_in_cavity_server : ' + goal.peg)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
            print client.get_result()
        except:
            pass
    else:
        rospy.logerr('Arguments were not received in the proper format !')
        rospy.loginfo('usage : insert OBJECT_NAME')
