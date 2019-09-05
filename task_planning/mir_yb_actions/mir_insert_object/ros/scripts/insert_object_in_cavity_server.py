#!/usr/bin/python
import sys
import rospy
import smach
import smach_ros

from std_msgs.msg import String

# import of generic states
import mir_states.common.manipulation_states as gms

# for dynamic reconfigure node + grasp monitor
import mcr_states.common.basic_states as gbs
# action lib
from smach_ros import ActionServerWrapper
import actionlib

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import InsertObjectAction
from mir_yb_action_msgs.msg import InsertObjectFeedback
from mir_yb_action_msgs.msg import InsertObjectResult

from mir_yb_action_msgs.msg import UnStageObjectAction
from mir_yb_action_msgs.msg import UnStageObjectGoal


#For wiggle arm
import moveit_commander 
import moveit_msgs.msg
from control_msgs.msg import GripperCommandActionGoal

arm_command = moveit_commander.MoveGroupCommander('arm_1')
arm_command.set_goal_position_tolerance(0.01)
arm_command.set_goal_orientation_tolerance(0.01)
arm_command.set_goal_joint_tolerance(0.005)        


#===============================================================================
class unstage(smach.State): # inherit from the State base class
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                                   input_keys=['insert_object_goal'], 
                                   output_keys=['insert_object_feedback', 'insert_object_result', 'move_arm_to'])

        self.client = actionlib.SimpleActionClient('unstage_object_server', UnStageObjectAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = UnStageObjectGoal()
        goal.robot_platform = userdata.insert_object_goal.robot_platform
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(15.0))
        result = self.client.get_result()
        if result.success == False:
            return 'failed'
        else:
            return 'succeeded'

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
        result = InsertObjectResult()
        result.success = False
        userdata.insert_object_result = result
        # give feedback
        feedback = InsertObjectFeedback()
        feedback.current_state = 'SELECT_OBJECT'
        userdata.insert_object_feedback = feedback
        # receive parameters from actionlib
        object_to_place = userdata.insert_object_goal.peg.upper()
        # creating string message
        msg = String()
        # filling message
        msg.data = object_to_place
        self.publisher.publish(msg)
        # do not kill the node so fast, let the topic to survive for some time
        rospy.loginfo('publishing on ' + self.topic_name + ' : ' + object_to_place)
        rospy.sleep(0.2)
        return 'success'
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
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

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
        elif self.callback_msg_.data == 'e_success' or self.callback_msg_.data == 'e_selected':
            self.callback_msg_ = None
            return 'success'
        else:
            print('[wait for event] : no response, or response message "{}" is not known.'.format(self.callback_msg_.data))
            self.callback_msg_ = None
            return 'failure'
        
    def execute(self, userdata):
        # give feedback
        feedback = InsertObjectFeedback()
        feedback.current_state = 'WAIT_FOR_FEEDBACK'
        userdata.insert_object_feedback = feedback
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
class ppt_wiggle_arm(smach.State):
    """
    Wiggle arm after placement on the table 
    """
    def __init__(self, wiggle_offset=0.0, joint=0):
        smach.State.__init__(self,
                             outcomes=['succeeded','failed'])
        self.wiggle_offset = wiggle_offset
        self.joint_number = joint
        self.blocking = True

        # create publisher for sending gripper position
        self.topic_name = '/arm_1/gripper_controller/gripper_command/goal'
        
        self.publisher = rospy.Publisher(self.topic_name, GripperCommandActionGoal, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def execute(self, userdata):
        #open the arm slightly 
        command = GripperCommandActionGoal()
        command.goal.command.position = 0.05
        self.publisher.publish(command)

        joint_values = arm_command.get_current_joint_values()
        joint_values[self.joint_number] = joint_values[self.joint_number] + self.wiggle_offset
        try:
            arm_command.set_joint_value_target(joint_values)               
        except Exception as e:
            rospy.logerr('unable to set target position: %s' % (str(e)))
            return 'failed'
        
        error_code = arm_command.go(wait=self.blocking)

        if error_code == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return 'succeeded'
        else:
            rospy.logerr("Arm movement failed with error code: %d", error_code)
            return 'failed'
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
        
class grasp_monitor_mockup(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['insert_object_goal'],
                                    output_keys=['insert_object_feedback', 'insert_object_result'])
    
    def execute(self, userdata):
        # mockup graps monitor, will always return success -> replace with padmaja stuff later on
        return 'succeeded'

#===============================================================================

def main(mokeup=False):
    # Open the container
    rospy.init_node('insert_object_in_cavity_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['insert_object_goal'],
            output_keys = ['insert_object_feedback', 'insert_object_result'])
    with sm:

        # publish object as string to mcr_perception_selectors -> cavity, this component then publishes
        # pose in base_link reference frame
        smach.StateMachine.add('SELECT_OBJECT', select_object('/mcr_perception/cavity_pose_selector/object_name'),
            transitions={'success':'CHECK_IF_OBJECT_IS_AVAILABLE'})

        smach.StateMachine.add('CHECK_IF_OBJECT_IS_AVAILABLE', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/cavity_pose_selector/event_in','e_trigger')],
                event_out_list=[('/mcr_perception/cavity_pose_selector/event_out','e_success', True)],
                timeout_duration=10),
                transitions={'success':'PLAN_WHOLE_BODY_MOTION',
                            'timeout':'SET_ACTION_LIB_FAILURE',
                            'failure':'SET_ACTION_LIB_FAILURE'})

        smach.StateMachine.add('PLAN_WHOLE_BODY_MOTION', send_event('/whole_body_motion_calculator_pipeline/event_in','e_start'),
            transitions={'success':'WAIT_PLAN_WHOLE_BODY_MOTION'})

        # wait for the result of the pregrasp planner
        smach.StateMachine.add('WAIT_PLAN_WHOLE_BODY_MOTION', wait_for_event('/whole_body_motion_calculator_pipeline/event_out', 15.0),
            transitions={'success':'CALCULATE_BASE_MOTION',
                          'timeout': 'STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE',
                          'failure':'STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE'})
        
        # pregrasp planner failed or timeout, stop the component and then return overall failure
        smach.StateMachine.add('STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE', send_event('/whole_body_motion_calculator_pipeline/event_in','e_stop'),
            transitions={'success':'SELECT_PREGRASP_PLANNER_INPUT_2'})  # go to  select pose input and plan arm motion

        smach.StateMachine.add('CALCULATE_BASE_MOTION', gbs.send_and_wait_events_combined(
                event_in_list=[('/base_motion_calculator/event_in','e_start')],
                event_out_list=[('/base_motion_calculator/event_out','e_success', True)],
                timeout_duration=5),
                transitions={'success':'STOP_PLAN_WHOLE_BODY_MOTION',
                             'timeout':'STOP_MOVE_BASE_TO_OBJECT',
                             'failure':'STOP_MOVE_BASE_TO_OBJECT'})

        # wbc pipeline  was successful, so lets stop it since its work is done
        smach.StateMachine.add('STOP_PLAN_WHOLE_BODY_MOTION', send_event('/whole_body_motion_calculator_pipeline/event_in','e_stop'),
            transitions={'success':'UNSTAGE_OBJECT'})
        
        smach.StateMachine.add('UNSTAGE_OBJECT', unstage(),
                transitions={'succeeded': 'MOVE_ROBOT_TO_OBJECT',
                             'failed' : 'SET_ACTION_LIB_FAILURE'})                            

        # execute robot motion
        smach.StateMachine.add('MOVE_ROBOT_TO_OBJECT', gbs.send_and_wait_events_combined(
                event_in_list=[ ('/wbc/mcr_navigation/direct_base_controller/coordinator/event_in','e_start')],
                event_out_list=[('/wbc/mcr_navigation/direct_base_controller/coordinator/event_out','e_success', True)],
                timeout_duration=5),
                transitions={'success':'STOP_MOVE_BASE_TO_OBJECT',
                             'timeout':'STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE',
                             'failure':'STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE'})

        # send stop event_in to arm motion component
        smach.StateMachine.add('STOP_MOVE_BASE_TO_OBJECT', 
                        send_event('/wbc/mcr_navigation/direct_base_controller/coordinator/event_in','e_stop'),
            transitions={'success':'SELECT_PREGRASP_PLANNER_INPUT_2'})

        # send stop event_in to arm motion component and return failure
        smach.StateMachine.add('STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE', 
                        send_event('/wbc/mcr_navigation/direct_base_controller/coordinator/event_in','e_stop'),
            transitions={'success':'SELECT_PREGRASP_PLANNER_INPUT_2'})

       ########################################## PREGRASP PIPELINE 2 ##################################################

        smach.StateMachine.add('SELECT_PREGRASP_PLANNER_INPUT_2', gbs.send_event([('/mir_manipulation/mux_pregrasp_pose/select',
                                                                    '/mcr_perception/object_selector/output/object_pose')]),
                transitions={'success':'SELECT_OBJECT_2'})

        smach.StateMachine.add('SELECT_OBJECT_2', select_object('/mcr_perception/cavity_pose_selector/object_name'),
            transitions={'success':'CHECK_IF_OBJECT_IS_AVAILABLE_2'})

        smach.StateMachine.add('CHECK_IF_OBJECT_IS_AVAILABLE_2', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/cavity_pose_selector/event_in','e_trigger')],
                event_out_list=[('/mcr_perception/cavity_pose_selector/event_out','e_success', True)],
                timeout_duration=10),
                transitions={'success':'PLAN_ARM_MOTION_2',
                            'timeout':'SET_ACTION_LIB_FAILURE',
                            'failure':'SET_ACTION_LIB_FAILURE'})

        smach.StateMachine.add('PLAN_ARM_MOTION_2', gbs.send_event([('/pregrasp_planner_pipeline/event_in','e_start')]),
            transitions={'success':'WAIT_PLAN_ARM_MOTION_2'})
        
        # wait for the result of the pregrasp planner
        smach.StateMachine.add('WAIT_PLAN_ARM_MOTION_2', wait_for_event('/pregrasp_planner_pipeline/event_out', 15.0),
           transitions={'success':'STOP_PLAN_ARM_MOTION_2',
                            'timeout': 'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2',
                            'failure':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2'})

        smach.StateMachine.add('STOP_PLAN_ARM_MOTION_2', gbs.send_event([('/pregrasp_planner_pipeline/event_in','e_stop')]),
            transitions={'success':'MOVE_ARM_TO_OBJECT_2'})
        
        # execute robot motion
        smach.StateMachine.add('MOVE_ARM_TO_OBJECT_2', gbs.send_and_wait_events_combined(
                #event_in_list=[('/move_arm_planned/event_in','e_start')],
                #event_out_list=[('/move_arm_planned/event_out','e_success', True)],
                event_in_list=[('/waypoint_trajectory_generation/event_in','e_start')],
                event_out_list=[('/waypoint_trajectory_generation/event_out','e_success', True)],
                timeout_duration=10),
                transitions={'success':'STOP_MOVE_ARM_TO_OBJECT_2',
                             'timeout':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2',
                             'failure':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2'})
        
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT_2', gbs.send_event([('/waypoint_trajectory_generation/event_out','e_stop')]),
            transitions={'success':'OPEN_GRIPPER'})
        
        #smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECTH_FAILURE', send_event('/move_arm_planned/event_in','e_stop'),
        ################################ When failure place the object 
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE_2', gbs.send_event([('/waypoint_trajectory_generation/event_out','e_stop')]),
            transitions={'success':'MOVE_ARM_TO_DEFAULT_PLACE'})

        smach.StateMachine.add('MOVE_ARM_TO_DEFAULT_PLACE', gms.move_arm("10cm/pose4"), 
                               transitions={'succeeded':'OPEN_GRIPPER_DEFAULT',
                                            'failed':'MOVE_ARM_TO_DEFAULT_PLACE'})

        smach.StateMachine.add('OPEN_GRIPPER_DEFAULT', gms.control_gripper('open'),
                transitions={'succeeded': 'MOVE_ARM_BACK'})

        smach.StateMachine.add('MOVE_ARM_BACK', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'SET_ACTION_LIB_FAILURE', 
                                            'failed':'SET_ACTION_LIB_FAILURE'})

        ################################
        # close gripper
        smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open'),
                transitions={'succeeded': 'WIGGLE_ARM_LEFT'})


        ###################################
        # wiggling the arm for precision placement
        smach.StateMachine.add('WIGGLE_ARM_LEFT', ppt_wiggle_arm(wiggle_offset=-0.12, joint=0),
                transitions={'succeeded':'WIGGLE_ARM_RIGHT',
                             'failed':'WIGGLE_ARM_RIGHT'})

        smach.StateMachine.add('WIGGLE_ARM_RIGHT', ppt_wiggle_arm(wiggle_offset=0.24, joint=0),
            transitions={'succeeded':'WIGGLE_ARM_FORWARD',
                         'failed':'WIGGLE_ARM_FORWARD'})

        # Now moving perpendicular 
        smach.StateMachine.add('WIGGLE_ARM_FORWARD', ppt_wiggle_arm(wiggle_offset=-0.12, joint=3),
                transitions={'succeeded':'WIGGLE_ARM_BACKWARD',
                             'failed':'WIGGLE_ARM_BACKWARD'})

        smach.StateMachine.add('WIGGLE_ARM_BACKWARD', ppt_wiggle_arm(wiggle_offset=0.24, joint=3),
            transitions={'succeeded':'MOVE_ARM_TO_HOLD',
                         'failed':'MOVE_ARM_TO_HOLD'})
        ###################################

        # move arm to HOLD position
        smach.StateMachine.add('MOVE_ARM_TO_HOLD', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'SET_ACTION_LIB_SUCCESS', 
                                            'failed':'MOVE_ARM_TO_HOLD'})
        
        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
                                transitions={'succeeded':'RESET_PREGRASP_PLANNER_INPUT_SUCCESS'})

        smach.StateMachine.add('RESET_PREGRASP_PLANNER_INPUT_SUCCESS', send_event('/mir_manipulation/mux_pregrasp_pose/select',
                                                                   '/mcr_perception/object_selector/output/object_pose'),
                                transitions={'success':'OVERALL_SUCCESS'})
                               
        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
                               transitions={'succeeded':'MOVE_ARM_TO_HOLD_FAILURE'})
 
        smach.StateMachine.add('MOVE_ARM_TO_HOLD_FAILURE', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'OVERALL_FAILED', 
                                            'failed':'MOVE_ARM_TO_HOLD_FAILURE'})
    
    # smach viewer
    sis = smach_ros.IntrospectionServer('insert_object_smach_viewer', sm, '/INSERT_OBJECT_IN_CAVITY_SMACH_VIEWER')
    sis.start()
    
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'insert_object_in_cavity_server',
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
    if len(sys.argv) > 1:
       main(mokeup=(sys.argv[1]=='mokeup'))
    else:
       main(mokeup=False)
