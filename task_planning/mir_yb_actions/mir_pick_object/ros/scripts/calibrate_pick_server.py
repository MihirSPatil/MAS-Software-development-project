#!/usr/bin/python
import sys
import rospy
import smach
import smach_ros

from std_msgs.msg import String
import geometry_msgs.msg

# import of generic states
import mir_states.common.manipulation_states as gms

# for dynamic reconfigure node + grasp monitor
import mcr_states.common.basic_states as gbs
# action lib
from smach_ros import ActionServerWrapper

from mir_yb_action_msgs.msg import PickObjectWBCAction
from mir_yb_action_msgs.msg import PickObjectWBCFeedback
from mir_yb_action_msgs.msg import PickObjectWBCResult
        
#===============================================================================

class SetupMoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['success'],
                                    input_keys=['pick_object_wbc_goal', 'move_arm_to'],
                                    output_keys=['pick_object_wbc_feedback', 'pick_object_wbc_result', 'move_arm_to'])

    def execute(self, userdata):
        result = PickObjectWBCResult()
        result.success = False
        userdata.pick_object_wbc_result = result
        # give feedback
        feedback = PickObjectWBCFeedback()
        feedback.current_state = 'SELECT_OBJECT'
        userdata.pick_object_wbc_feedback = feedback
        # the goal will contain the arm location to move
        userdata.move_arm_to = userdata.pick_object_wbc_goal.object
        # do not kill the node so fast, let the topic to survive for some time
        rospy.loginfo('move arm to : ' + userdata.move_arm_to)
        return 'success'
        
#===============================================================================
class WaitForEventFromUser(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo("Have you placed the mockup pose press y/n ")
        input_from_user = raw_input()
        if ('y' == input_from_user):
	    rospy.loginfo("Received YES .. moving arm ")
            return 'success'
        else:
	    rospy.loginfo("Received NO .. EXITING MOVING ARM ")
            return 'failure'
#===============================================================================

class send_event(smach.State):
    '''
    This class publishes e_start on topic_name argument
    '''
    def __init__(self, topic_name, event):
        smach.State.__init__(self,  outcomes=['success'],
                                    input_keys=['pick_object_wbc_goal'],
                                    output_keys=['pick_object_wbc_feedback', 'pick_object_wbc_result'])
        # create publisher
        self.topic_name = topic_name
        self.event = event
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def execute(self, userdata):
        # give feedback
        feedback = PickObjectWBCFeedback()
        feedback.current_state = 'SEND_EVENT'
        userdata.pick_object_wbc_feedback = feedback
        # creating string message
        msg = String()
        # filling message
        msg.data = self.event
        # publish
        self.publisher.publish(msg)
        rospy.loginfo('publishing on ' + self.topic_name + ' ' + self.event)
        # wait, dont kill the node so quickly
        rospy.sleep(0.2)
        return 'success'
        
#===============================================================================

class wait_for_event(smach.State):
    '''
    This state will take a event name as input and waits for the event to
    be published.
    '''
    def __init__(self, topic_name, timeout_duration):
        smach.State.__init__(self,  outcomes=['success', 'failure', 'timeout'],
                                    input_keys=['pick_object_wbc_goal'],
                                    output_keys=['pick_object_wbc_feedback', 'pick_object_wbc_result'])
        rospy.Subscriber(topic_name, String, self.event_cb)
        self.callback_msg_ = None
        self.message_received = False
        self.timeout = rospy.Duration.from_sec(timeout_duration)

    def event_cb(self, callback_msg):
        self.callback_msg_ = callback_msg
        self.message_received = True

    def getResult(self):
        if self.callback_msg_ is None:
            rospy.logerr('event out message not received in the specified time')
            return 'timeout'
        elif self.callback_msg_.data == 'e_failure':
            self.callback_msg_ = None
            return 'failure'
        elif self.callback_msg_.data == 'e_success' or self.callback_msg_.data == 'e_selected':
            self.callback_msg_ = None
            return 'success'
        else:
            print('[wait for event] : no response, or response message "{}" is not known.'.format(self.callback_msg_.data))
            self.callback_msg_ = None
            return 'failure'
        
    def execute(self, userdata):
        # give feedback
        feedback = PickObjectWBCFeedback()
        feedback.current_state = 'WAIT_FOR_FEEDBACK'
        userdata.pick_object_wbc_feedback = feedback
        # reset flag of received to false
        print "waiting for node response..."
        self.message_received = False
        start_time = rospy.Time.now()
        rate = rospy.Rate(10) # 10hz
        # wait for message to arrive
        while((rospy.Time.now() - start_time < self.timeout) and not(self.message_received) and not(rospy.is_shutdown())):
            rate.sleep()
        # dont kill the node so quickly, wait for handshake of nodes
        print("(rospy.Time.now() - start_time < self.timeout) = {}".format((rospy.Time.now() - start_time < self.timeout)))
        print("self.message_received = {}".format((self.message_received)))
        rospy.sleep(0.2)  
        return self.getResult()

        
#===============================================================================
        
class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['pick_object_wbc_goal'],
                                    output_keys=['pick_object_wbc_feedback', 'pick_object_wbc_result'])
        self.result = result

    def execute(self, userdata):
        result = PickObjectWBCResult()
        result.success = self.result
        userdata.pick_object_wbc_result = result
        return 'succeeded'
        
        
#===============================================================================
        
class grasp_monitor_mockup(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['pick_object_wbc_goal'],
                                    output_keys=['pick_object_wbc_feedback', 'pick_object_wbc_result'])
    
    def execute(self, userdata):
        # mockup graps monitor, will always return success -> replace with padmaja stuff later on
        return 'succeeded'

class publish_object_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded', 'failed'])
        object_topic = "/mcr_perception/object_selector/output/object_pose"

        rospy.Subscriber(object_topic, geometry_msgs.msg.PoseStamped, self.object_pose_cb)
        self.pose_pub = rospy.Publisher(object_topic, geometry_msgs.msg.PoseStamped)
        self.object_pose = None

    def object_pose_cb(self, msg):
        
        self.object_pose = msg

    def execute(self, userdata):
        rospy.sleep(2.0)        
        if self.object_pose is not None:
           self.pose_pub.publish(self.object_pose)
           self.object_pose = None
           return 'succeeded'
        return 'failed'
#===============================================================================

def main(mokeup=False):
    # Open the container
    rospy.init_node('pick_object_wbc_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['pick_object_wbc_goal'],
            output_keys = ['pick_object_wbc_feedback', 'pick_object_wbc_result'])
    with sm:

        if not mokeup:
            # open gripper
            smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open'),
                transitions={'succeeded': 'SETUP_MOVE_ARM'})
            
        #add states to the container
        smach.StateMachine.add('SETUP_MOVE_ARM', SetupMoveArm(),
                transitions={'success': 'MOVE_ARM'})

        smach.StateMachine.add('MOVE_ARM', gms.move_arm(),
                transitions={'succeeded': 'WAIT_FOR_EVENT_FROM_USER',
                             'failed': 'MOVE_ARM'})

        smach.StateMachine.add('WAIT_FOR_EVENT_FROM_USER', WaitForEventFromUser(),
                 transitions={'success' : 'PLAN_WHOLE_BODY_MOTION',
			      'failure' : 'STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE'})

        smach.StateMachine.add('PLAN_WHOLE_BODY_MOTION', send_event('/whole_body_motion_calculator_pipeline/event_in','e_start'),
            transitions={'success':'WAIT_PLAN_WHOLE_BODY_MOTION'})

        # wait for the result of the pregrasp planner
        smach.StateMachine.add('WAIT_PLAN_WHOLE_BODY_MOTION', wait_for_event('/whole_body_motion_calculator_pipeline/event_out', 15.0),
            transitions={'success':'STOP_PLAN_WHOLE_BODY_MOTION',
                          'timeout': 'STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE',
                          'failure':'STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE'})
 
        # pregrasp planner failed or timeout, stop the component and then return overall failure
        smach.StateMachine.add('STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE', send_event('/whole_body_motion_calculator_pipeline/event_in','e_stop'),
            transitions={'success':'SELECT_PREGRASP_PLANNER_INPUT_2'})  # go to  select pose input and plan arm motion

        smach.StateMachine.add('STOP_PLAN_WHOLE_BODY_MOTION', send_event('/whole_body_motion_calculator_pipeline/event_in','e_stop'),
            transitions={'success':'SELECT_PREGRASP_PLANNER_INPUT_2'})  # go to  select pose input and plan arm motion

       ########################################## PREGRASP PIPELINE 2 ##################################################
        # based on a published pose, calls pregrasp planner to generate a graspable pose
        smach.StateMachine.add('SELECT_PREGRASP_PLANNER_INPUT_2', send_event('/pregrasp_planner_pipeline/event_in','e_start'),
            transitions={'success':'WAIT_PLAN_ARM_MOTION_2'})
        
        # wait for the result of the pregrasp planner
        smach.StateMachine.add('WAIT_PLAN_ARM_MOTION_2', wait_for_event('/pregrasp_planner_pipeline/event_out', 15.0),
           transitions={'success':'STOP_PLAN_ARM_MOTION_2',
                            'timeout': 'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2',
                            'failure':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2'})

        smach.StateMachine.add('STOP_PLAN_ARM_MOTION_2', send_event('/pregrasp_planner_pipeline/event_in','e_stop'),
            transitions={'success':'MOVE_ARM_TO_OBJECT_2'})
        
        # execute robot motion
        smach.StateMachine.add('MOVE_ARM_TO_OBJECT_2', gbs.send_and_wait_events_combined(
                event_in_list=[('/waypoint_trajectory_generation/event_in','e_start')],
                event_out_list=[('/waypoint_trajectory_generation/event_out','e_success', True)],
                timeout_duration=10),
                transitions={'success':'STOP_MOVE_ARM_TO_OBJECT_2',
                             'timeout':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2',
                             'failure':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2'})
        
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT_2', send_event('/waypoint_trajectory_generation/event_out','e_stop'),
            transitions={'success':'SET_ACTION_LIB_SUCCESS'})
        
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2', send_event('/waypoint_trajectory_generation/event_out','e_stop'),
            transitions={'success':'SET_ACTION_LIB_FAILURE'})
        
        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
                                transitions={'succeeded':'OVERALL_SUCCESS'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
                               transitions={'succeeded':'MOVE_ARM_TO_HOLD_FAILURE'})
 
        smach.StateMachine.add('MOVE_ARM_TO_HOLD_FAILURE', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'OVERALL_FAILED', 
                                            'failed':'OVERALL_FAILED'})
    # smach viewer
    sis = smach_ros.IntrospectionServer('calibrate_pick_object_smach_viewer', sm, '/CALIBRATE_PICK_OBJECT_SMACH_VIEWER')
    sis.start()
    
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'calibrate_pick_server',
        action_spec = PickObjectWBCAction,
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'pick_object_wbc_goal',
        feedback_key = 'pick_object_wbc_feedback',
        result_key   = 'pick_object_wbc_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()
        
if __name__ == '__main__':
    if len(sys.argv) > 1:
       main(mokeup=(sys.argv[1]=='mokeup'))
    else:
       main(mokeup=False)
