#!/usr/bin/env python
import rospy
import smach

import smach_ros

import mir_states.common.manipulation_states as gms # move the arm, and gripper
        
# knowledge base update
#from knowledge_base_sm import UpdateKnowledgeBase

from std_msgs.msg import String

# take string and publish even_in to dynamic reconfigure and wait for event out until reconfigure is done
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import InsertObjectAction
from mir_yb_action_msgs.msg import InsertObjectFeedback
from mir_yb_action_msgs.msg import InsertObjectResult

## NOTE : keep in mind that you need to send gripper to close with moveit before executing this
## action lib, otherwise gripper will not open

#===============================================================================

class select_object(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(self,  outcomes=['success'],
                                    input_keys=['insert_object_goal'], 
                                    output_keys=['insert_object_feedback', 'insert_object_result'])
        # create publisher
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def execute(self, userdata):
        # updating result (false until finished all actions)
        result = InsertObjectResult()
        result.success = False
        userdata.insert_object_result = result
        # give feedback
        feedback = InsertObjectFeedback()
        feedback.current_state = 'SELECT_OBJECT'
        userdata.insert_object_feedback = feedback
        # receive parameters from actionlib
        object_to_insert_into = userdata.insert_object_goal.hole
        # creating string message
        msg = String()
        # filling message
        msg.data = object_to_insert_into
        #msg.data = 'EM-02'
        #msg.data = 'ER-02'
        self.publisher.publish(msg)
        # do not kill the node so fast, let the topic to survive for some time
        rospy.loginfo('publishing on ' + self.topic_name + ' : '+ object_to_insert_into)
        rospy.sleep(0.2)
        return 'success'
        
#===============================================================================           
        
class MoveArmTopHole(smach.State): # inherit from the State base class
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded','failed'], 
                                    input_keys=['insert_object_goal'], 
                                    output_keys=['insert_object_feedback', 'insert_object_result'])
        self.counter = 0

    def execute(self, userdata):
        return 'success'       #mockup TODO!!  
        
#===============================================================================

class send_event(smach.State):
    '''
    This class publishes e_start on topic_name argument
    '''
    def __init__(self, topic_name, event):
        smach.State.__init__(self,  outcomes=['success'],
                                    input_keys=['insert_object_goal'], 
                                    output_keys=['insert_object_feedback', 'insert_object_result'])
        # create publisher
        self.topic_name = topic_name
        self.event = event
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=1)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.5)

    def execute(self, userdata):
        # give feedback
        feedback = InsertObjectFeedback()
        feedback.current_state = 'SEND_EVENT'
        userdata.insert_object_feedback = feedback
        # creating string message
        msg = String()
        # filling message
        msg.data = self.event
        # publish
        self.publisher.publish(msg)
        rospy.loginfo('publishing on ' + self.topic_name + ' ' + self.event)
        return 'success'
        
#===============================================================================

class wait_for_event(smach.State):
    '''
    This state will take a event name as input and waits for the event to
    be published.
    '''
    def __init__(self, topic_name, timeout_duration):
        smach.State.__init__(self,  outcomes=['success', 'failure', 'timeout'],
                                    input_keys=['insert_object_goal'], 
                                    output_keys=['insert_object_feedback', 'insert_object_result'])
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
        elif self.callback_msg_.data == 'e_success':
            self.callback_msg_ = None
            return 'success'
        print '[wait for event] : no response, or response message is not known'
        return 'failure'
        
    def execute(self, userdata):
        # give feedback
        feedback = InsertObjectFeedback()
        feedback.current_state = 'WAIT_FOR_FEEDBACK'
        userdata.insert_object_feedback = feedback
        # reset flag of received to false
        self.message_received = False
        start_time = rospy.Time.now()
        rate = rospy.Rate(10) # 10hz
        # wait for message to arrive
        while((rospy.Time.now() - start_time < self.timeout) and not(self.message_received) and not(rospy.is_shutdown())):
            rate.sleep()
        self.message_received = False        
        return self.getResult()
        
#=============================================================================== 
       
class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'], 
                                    input_keys=['insert_object_goal'], 
                                    output_keys=['insert_object_feedback', 'insert_object_result'])
        self.result = result

    def execute(self, userdata):
        result = InsertObjectResult()
        result.success = self.result
        userdata.insert_object_result = result
        return 'succeeded'
        
        
#===============================================================================

def main():
    rospy.init_node('smach_example_state_machine')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['insert_object_goal'],
            output_keys = ['insert_object_feedback', 'insert_object_result'])
    with sm:
        # publish object as string to mcr_perception_selectors -> object_selector, this component then publishes
        # pose in base_link reference frame when e_trigger is sent (next state)
        smach.StateMachine.add('SELECT_OBJECT', select_object('/mcr_perception/object_selector/input/object_name'),
            transitions={'success':'GENERATE_OBJECT_POSE'})
        
        # generates a pose based on the previous string object topic received
        smach.StateMachine.add('GENERATE_OBJECT_POSE', send_event('/mcr_perception/object_selector/event_in', 'e_trigger'),
            transitions={'success':'SET_TOP_HOLE_CONFIG'})
        
        # reconfigure top hole shifter
        smach.StateMachine.add('SET_TOP_HOLE_CONFIG', gbs.set_named_config('top_hole_shifter'),
            transitions={'success':'GENERATE_TOP_HOLE_POSE',
                         'failure':'SET_ACTION_LIB_FAILURE',
                         'timeout': 'SET_TOP_HOLE_CONFIG'})
        
        # receive a pose and offset's up, result: top hole pose (another pose)
        smach.StateMachine.add('GENERATE_TOP_HOLE_POSE', send_event('/top_hole_pose_shifter/event_in','e_start'),
            transitions={'success':'WAIT_FOR_CREATE_TOP_HOLE_POSE'})
        
        # wait for pose shifter to give response
        smach.StateMachine.add('WAIT_FOR_CREATE_TOP_HOLE_POSE', wait_for_event('/top_hole_pose_shifter/event_out', timeout_duration=5.0),
            transitions={'success':'SET_TOP_HOLE_PREGRASP_CONFIG',
                            'timeout':'SET_ACTION_LIB_FAILURE',
                            'failure':'SET_ACTION_LIB_FAILURE'})
        
        # reconfigure top hole pregrasp params
        smach.StateMachine.add('SET_TOP_HOLE_PREGRASP_CONFIG', gbs.set_named_config('top_hole_pregrasp'),
            transitions={'success':'PLAN_ARM_MOTION',
                         'failure':'SET_ACTION_LIB_FAILURE',
                         'timeout': 'SET_TOP_HOLE_PREGRASP_CONFIG'})
        
        # based on a published pose, calls pregrasp planner to generate a graspable pose
        smach.StateMachine.add('PLAN_ARM_MOTION', send_event('/top_hole_pregrasp_planner_pipeline/event_in','e_start'),
            transitions={'success':'WAIT_PLAN_ARM_MOTION'})
        
        # wait for the result of the pregrasp planner
        smach.StateMachine.add('WAIT_PLAN_ARM_MOTION', wait_for_event('/top_hole_pregrasp_planner_pipeline/event_out', 3.0),
            transitions={'success':'STOP_PLAN_ARM_MOTION',
                            'timeout': 'STOP_PLAN_ARM_MOTION_WITH_FAILURE',
                            'failure':'STOP_PLAN_ARM_MOTION_WITH_FAILURE'})
        
        # pregrasp planner was successful, so lets stop it since its work is done
        smach.StateMachine.add('STOP_PLAN_ARM_MOTION', send_event('/top_hole_pregrasp_planner_pipeline/event_in','e_stop'),
            transitions={'success':'MOVE_ARM_TO_OBJECT'})
        
        
        # pregrasp planner failed or timeout, stop the component and then return overall failure
        smach.StateMachine.add('STOP_PLAN_ARM_MOTION_WITH_FAILURE', send_event('/top_hole_pregrasp_planner_pipeline/event_in','e_stop'),
            transitions={'success':'SET_ACTION_LIB_FAILURE'})
        
        # move arm to pregrasp planned pose
        smach.StateMachine.add('MOVE_ARM_TO_OBJECT', send_event('/top_hole_move_arm_planned/event_in','e_start'),
            transitions={'success':'WAIT_MOVE_ARM_TO_OBJECT'})
        
        # wait for the arm motion to finish
        smach.StateMachine.add('WAIT_MOVE_ARM_TO_OBJECT', wait_for_event('/top_hole_move_arm_planned/event_out', 10.0),
            transitions={'success':'STOP_MOVE_ARM_TO_OBJECT',
                            'timeout': 'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE',
                            'failure':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE'})
        
        # send stop event_in to arm motion component and return failure
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE', send_event('/top_hole_move_arm_planned/event_in','e_stop'),
            transitions={'success':'SET_ACTION_LIB_FAILURE'})
        
        # send stop event_in to arm motion component
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT', send_event('/top_hole_move_arm_planned/event_in','e_stop'),
            transitions={'success':'CONFIGURE_POSES_TO_MOVE_DOWN'})
                                   
        # reconfigure the pose
        smach.StateMachine.add('CONFIGURE_POSES_TO_MOVE_DOWN', gbs.set_named_config('move_down_shifter'),
            transitions={'success':'PLAN_LINEAR_MOTION_DOWN',
                         'failure':'SET_ACTION_LIB_FAILURE',
                         'timeout': 'CONFIGURE_POSES_TO_MOVE_DOWN'})
                                   
        # linear motion down states
        smach.StateMachine.add('PLAN_LINEAR_MOTION_DOWN', send_event('/poses_to_move/event_in', 'e_start'),
            transitions={'success':'WAIT_FOR_PLAN_LINEAR_MOTION_DOWN'})
        
        smach.StateMachine.add('WAIT_FOR_PLAN_LINEAR_MOTION_DOWN', wait_for_event('/poses_to_move/event_out', timeout_duration=5.0),
            transitions={'success':'MOVE_TO_CARTESIAN_POSE_DOWN',
                            'failure':'MOVE_ARM_TO_HOLD_FAILED',
                            'timeout' : 'MOVE_ARM_TO_HOLD_FAILED'})
        
        # linear motion common states
        smach.StateMachine.add('MOVE_TO_CARTESIAN_POSE_DOWN', send_event('/cartesian_controller_demo/event_in','e_start'),
            transitions={'success':'WAIT_RESULT_MOVE_TO_CARTESIAN_POSE_DOWN'})
        
        smach.StateMachine.add('WAIT_RESULT_MOVE_TO_CARTESIAN_POSE_DOWN', wait_for_event('/cartesian_controller_demo/event_out', timeout_duration=5.0),
            transitions={'success':'OPEN_GRIPPER',
                            'timeout':'MOVE_ARM_TO_HOLD_FAILED',
                            'failure':'MOVE_ARM_TO_HOLD_FAILED'})
        
        # open gripper
        smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open'),
                transitions={'succeeded': 'OPEN_GRIPPER_AGAIN'})
        
        # open gripper again (replace later on with padmaja stuff)
        smach.StateMachine.add('OPEN_GRIPPER_AGAIN', gms.control_gripper('open'),
                transitions={'succeeded': 'CONFIGURE_POSES_TO_MOVE_UP'})
        
        #reconfigure
        smach.StateMachine.add('CONFIGURE_POSES_TO_MOVE_UP', gbs.set_named_config('move_up_shifter'),
            transitions={'success':'PLAN_LINEAR_MOTION_UP',
                         'failure':'SET_ACTION_LIB_FAILURE',
                         'timeout': 'CONFIGURE_POSES_TO_MOVE_UP'})
        
        # linear motion up states
        smach.StateMachine.add('PLAN_LINEAR_MOTION_UP', send_event('/poses_to_move/event_in','e_start'),
            transitions={'success':'WAIT_FOR_PLAN_LINEAR_MOTION_UP'})
        
        smach.StateMachine.add('WAIT_FOR_PLAN_LINEAR_MOTION_UP', wait_for_event('/poses_to_move/event_out', timeout_duration=5.0),
            transitions={'success':'MOVE_TO_CARTESIAN_POSE_UP',
                            'timeout':'MOVE_ARM_TO_HOLD',
                            'failure':'MOVE_ARM_TO_HOLD'})
        
        # linear motion common states
        smach.StateMachine.add('MOVE_TO_CARTESIAN_POSE_UP', send_event('/cartesian_controller_demo/event_in','e_start'),
            transitions={'success':'WAIT_RESULT_MOVE_TO_CARTESIAN_POSE_UP'})

        smach.StateMachine.add('WAIT_RESULT_MOVE_TO_CARTESIAN_POSE_UP', wait_for_event('/cartesian_controller_demo/event_out', timeout_duration=5.0),
            transitions={'success':'MOVE_ARM_TO_HOLD',
                            'timeout':'MOVE_ARM_TO_HOLD',
                            'failure':'MOVE_ARM_TO_HOLD'})
        
        # move arm to HOLD
        smach.StateMachine.add('MOVE_ARM_TO_HOLD', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'SET_ACTION_LIB_SUCCESS', 
                                            'failed':'MOVE_ARM_TO_HOLD'})
                                         
        # move arm to HOLD but provided that some old component failed
        smach.StateMachine.add('MOVE_ARM_TO_HOLD_FAILED', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'SET_ACTION_LIB_FAILURE', 
                                            'failed':'MOVE_ARM_TO_HOLD_FAILED'})
                                            
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
                               transitions={'succeeded':'OVERALL_SUCCESS'})
                               
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False),
                               transitions={'succeeded':'OVERALL_FAILED'})
    
    # smach viewer
    sis = smach_ros.IntrospectionServer('insert_object_smach_viewer', sm, '/INSERT_OBJECT_SMACH_VIEWER')
    sis.start()
    
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'insert_object_server', 
        action_spec = InsertObjectAction, 
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'insert_object_goal',
        feedback_key = 'insert_object_feedback',
        result_key   = 'insert_object_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()
