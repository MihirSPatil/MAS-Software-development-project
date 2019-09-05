#!/usr/bin/env python
import rospy
import smach
import actionlib
# move the arm and base
import mir_states.common.manipulation_states as gms # move the arm

# for send and receive event combined
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import AlignWithWorkspaceAction
from mir_yb_action_msgs.msg import AlignWithWorkspaceFeedback
from mir_yb_action_msgs.msg import AlignWithWorkspaceResult
from mir_navigation_msgs.msg import OrientToBaseAction, OrientToBaseActionGoal
import dynamic_reconfigure.server
import mir_align_with_workspace.cfg.WorkspaceDistanceConfig as WorkspaceDistanceConfig

#===============================================================================

class SetupAlignWithWorkspace(smach.State): # inherit from the State base class
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded','failed','preempted'],
                                    input_keys=['align_with_workspace_goal', 'align_with_workspace_result', 'align_with_workspace_feedback'],
                                    output_keys=['align_with_workspace_feedback', 'align_with_workspace_result'])

        self.client = actionlib.SimpleActionClient('/mir_navigation/base_placement/adjust_to_workspace', OrientToBaseAction)
        rospy.loginfo('Waiting for action server to start.')
        self.client.wait_for_server()
        self.received_align_result = False
        self.goal = OrientToBaseActionGoal()
        self.dynamic_reconfigure = dynamic_reconfigure.server.Server(WorkspaceDistanceConfig, self.reconfigure_callback)

    def reconfigure_callback(self,config,params):
        self.config = config
        return config

    def execute(self, userdata):
        if self.preempt_requested():
            rospy.logwarn('preemption requested!!!')
            self.recall_preempt()
            return 'preempted'

        userdata.align_with_workspace_result  = AlignWithWorkspaceResult()
        workspace_goal = userdata.align_with_workspace_goal

        platform ='/mcr_perception/place_pose_selector/'+workspace_goal.destination_location
        print  platform+"   workspace_goal"
        height = rospy.get_param(str(platform), None)
        if (height==0 or workspace_goal.destination_location == 'CB02' or workspace_goal.destination_location == 'CB01'):
            return 'succeeded'

        self.goal.goal.distance  =self.assign_threshold(height)
        timeout = 10.0
        rospy.loginfo('Action server started, sending goal')
        self.client.send_goal(self.goal.goal)
        self.client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        userdata.align_with_workspace_result = self.client.get_result()

        if(userdata.align_with_workspace_result is None):
            self.client.cancel_goal()
            return 'failed'

        feedback = AlignWithWorkspaceFeedback()
        feedback.current_state = 'ALIGN_BASE'
        feedback.text='[align_with_workspace] Aligning with the workspace: '
        userdata.align_with_workspace_feedback = feedback

        if(userdata.align_with_workspace_result):
            return 'succeeded'
        else:
            return 'failed'

    def assign_threshold(self,height):
        threshold = 0.7
        if height ==20:
            threshold = self.config.height_20
        elif height==15:
            threshold = self.config.height_15
        elif height ==10:
            threshold= self.config.height_10
        elif height ==5:
            threshold = self.config.height_5
        elif height ==50:
            threshold = self.config.height_50
        else:
            threshold = 0.07
        return threshold

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['align_with_workspace_goal', 'align_with_workspace_result'],
                                    output_keys=['align_with_workspace_feedback', 'align_with_workspace_result'])
        self.result = result

    def execute(self, userdata):
        result = AlignWithWorkspaceResult()
        result.success = self.result
        userdata.align_with_workspace_result = result
        return 'succeeded'


#===============================================================================

def main():
    rospy.init_node('align_with_workspace_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED','OVERALL_PREEMPTED'],
            input_keys = ['align_with_workspace_goal', 'align_with_workspace_result'],
            output_keys = ['align_with_workspace_feedback', 'align_with_workspace_result'])
    with sm:
        smach.StateMachine.add('ALIGN_BASE', SetupAlignWithWorkspace(),
                transitions={'succeeded': 'SET_ACTION_LIB_SUCCESS',
                             'failed': 'SET_ACTION_LIB_FAILURE',
                             'preempted':'OVERALL_PREEMPTED'})

        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True),
                               #transitions={'succeeded':'RESET_STATIC_TRANSFORM_FOR_PERCEPTION'})
                               transitions={'succeeded':'OVERALL_SUCCESS'})

        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False),
                               transitions={'succeeded':'OVERALL_FAILED'})

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'align_with_workspace_server',
        action_spec = AlignWithWorkspaceAction,
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['OVERALL_PREEMPTED'],
        goal_key     = 'align_with_workspace_goal',
        feedback_key = 'align_with_workspace_feedback',
        result_key   = 'align_with_workspace_result')
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()
