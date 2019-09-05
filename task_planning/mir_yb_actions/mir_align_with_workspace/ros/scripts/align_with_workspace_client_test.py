#! /usr/bin/env python
import rospy
import roslib
import actionlib

import sys

from mir_yb_action_msgs.msg import AlignWithWorkspaceAction
from mir_yb_action_msgs.msg import AlignWithWorkspaceFeedback
from mir_yb_action_msgs.msg import AlignWithWorkspaceResult
from mir_yb_action_msgs.msg import AlignWithWorkspaceActionGoal

if __name__ == '__main__':
    rospy.init_node('align_with_workspace_client_tester')
    client = actionlib.SimpleActionClient('/align_with_workspace_server', AlignWithWorkspaceAction)
    client.wait_for_server()
    goal = AlignWithWorkspaceActionGoal()

    if len(sys.argv) == 2: # 1 arguments was received : ok proceed
        try:
            goal.goal.destination_location = str(sys.argv[1])
            timeout = 10.0
            rospy.loginfo('Sending action lib goal to align_with_workspace_server, destination : ' + goal.goal.destination_location)
            client.send_goal(goal.goal)
            client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
            client.cancel_goal()
            server_result = client.get_result()

        except:
            pass
    else:
        rospy.logerr('Arguments were not received in the proper format !')
        rospy.loginfo('usage : align_with_workspace SOURCE DESTINATION')
