#!/usr/bin/python
import sys
import rospy
import actionlib
import smach
import smach_ros
import math
import std_msgs.msg
import dynamixel_msgs.msg# import of generic states
from mcr_perception_msgs.msg import ObjectList, Object
from std_msgs.msg import String
# generic states
import mir_states.common.basic_states as gbs_o
import mir_states.common.navigation_states as gns_o
import mir_states.common.manipulation_states as gms_o
import mir_states.common.perception_states as gps_o
import mir_states_common.robocup.referee_box_communication as rbc

# for dynamic reconfigure node + grasp monitor
import mcr_states.common.basic_states as gbs
import mir_states.robocup.basic_transportation_test_states as btts
import mir_states.robocup.referee_box_states as refbox
import mir_states.common.perception_mockup_util as perception_mockup_util

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import PickObjectAction
from mir_yb_action_msgs.msg import PickObjectFeedback
from mir_yb_action_msgs.msg import PickObjectResult
from mir_yb_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal
#===============================================================================

class move_base_safe(smach.State):
    def __init__(self, destination_location, timeout=120.0, action_server='move_base_safe_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])
        self.destination_location = destination_location
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, MoveBaseSafeAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        result = PickObjectResult()
        result.success = False
        userdata.pick_object_result = resultresult = PickObjectResult()
        result.success = False
        userdata.pick_object_result = result
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'MOVE_TO_CONVEYOR'
        userdata.pick_object_feedback = feedback
        goal = MoveBaseSafeGoal()
        goal.source_location = 'anywhere'
        goal.destination_location = self.destination_location
        #rospy.loginfo('Sending actionlib goal to ' + self.action_server + ', destination: ',
        #              goal.destination_location + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'

#===============================================================================
class my_pause(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])

    def execute(self, userdata):
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'WAIT_TO_CLOSE_GRIPPER'
        userdata.pick_object_feedback = feedback
        rospy.sleep(0.4)
        rospy.loginfo("I HAVE SLEPT")
        return 'succeeded'

class wait_for_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])
        self.result = None
        self.img_editor = rospy.Publisher(
        '/mcr_perception/edit_image/event_in',std_msgs.msg.String, queue_size=11)
        self.event_out = rospy.Publisher(
        '/mcr_perception/background_change_detection/event_in', std_msgs.msg.String, queue_size=10)
        rospy.Subscriber(
        '/mcr_perception/background_change_detection/event_out', std_msgs.msg.String, self.event_cb)

    def execute(self, userdata):
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'WAIT_FOR_OBJECT'
        userdata.pick_object_feedback = feedback
        self.result = None
        self.img_editor.publish('e_start')
        rospy.loginfo('published')
        ### TODO CHANGE THIS
        rospy.sleep(1)
        self.event_out.publish('e_start')
        rospy.sleep(1)
        while not self.result:
            rospy.sleep(0.1)
        self.img_editor.publish('e_stop')
        self.event_out.publish('e_stop')

        if self.result.data != 'e_change':
            return 'failed'
        return 'succeeded'

    def event_cb(self, msg):
        self.result = msg
#===============================================================================

class for_testing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])

    def execute(self, userdata):
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'MOVE_TO_CONVEYOR'
        userdata.pick_object_feedback = feedback
        rospy.sleep(1.0)
        if True:
            return 'succeeded'
        else:
            return 'failed'
#===============================================================================
class cbt_release(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])

    def execute(self, userdata):
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'OPEN_GRIPPER'
        userdata.pick_object_feedback = feedback
        gms_o.control_gripper('open_wide')
        #gms_o.gripper_command.go()
        rospy.sleep(3.0)
        return 'succeeded'

#===============================================================================
class cbt_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])

    def execute(self, userdata):
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'CBT_GRASP'
        userdata.pick_object_feedback = feedback
        gms_o.control_gripper('close')
        #gms_o.gripper_command.go()
        rospy.sleep(3.0)
        return 'succeeded'
#===============================================================================
class cbt_grasp_relese(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])

    def execute(self, userdata):
        #TODO TIMING
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'CBT_RELEASE'
        userdata.pick_object_feedback = feedback
        gms_o.control_gripper("open")
        #rospy.sleep(3.5)
        #gms_o.gripper_command.set_named_target("open")
        #gms_o.gripper_command.go()
        rospy.sleep(2.0)
        return 'succeeded'
#===============================================================================
class get_cbt_task(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['task_received'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])

        rospy.Subscriber("/robot_example_ros/task_info", atwork_ros_msgs.msg.TaskInfo, self.refboxcb)
        self.is_refbox_start_recieved = False

    def refboxcb(self, msg):
        self.is_refbox_start_recieved = True

    def execute(self, userdata):
        rospy.loginfo("Waiting for task")
        while not rospy.is_shutdown():
            if (self.is_refbox_start_recieved):
                break
            rospy.sleep(1.0)
        result = PickObjectResult()
        result.success = False
        userdata.pick_object_result = result
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'GET_TASK'
        userdata.pick_object_feedback = feedback
        return "task_received"
#===============================================================================

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])
        self.result = result

    def execute(self, userdata):
        result = PickObjectResult()
        result.success = self.result
        userdata.pick_object_result = result
        return 'succeeded'

#===============================================================================

class grasp_monitor_mockup(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['pick_object_goal'],
                                    output_keys=['pick_object_feedback', 'pick_object_result'])

    def execute(self, userdata):
        # mockup graps monitor, will always return success -> replace with padmaja stuff later on
        return 'succeeded'

#===============================================================================

def main(simulation=False):
    print "Starting node"
    rospy.init_node('pick_object_rtt_server')

    print "Starting StateMachine"
    SM = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['pick_object_goal'],
            output_keys = ['pick_object_feedback', 'pick_object_result'])

    # open the container

    use_mockup=True

    with SM:

        #smach.StateMachine.add('OPEN_GRIPPER_2', gms_o.control_gripper('close'),
         #   transitions={'succeeded': 'ARM_TO_CONVEYOR_VIEW'})


        #smach.StateMachine.add('INIT_ROBOT', gbs_o.init_robot(),
            #transitions={'succeeded':'GET_TASK'})
           # transitions={'succeeded':'ARM_TO_CONVEYOR_VIEW'})

        if use_mockup:
            smach.StateMachine.add('MOVE_TO_CONVEYOR', for_testing(),
                transitions={'succeeded': 'ARM_TO_CONVEYOR_VIEW',
                            'failed': 'ARM_TO_CONVEYOR_VIEW'})

        else:
            smach.StateMachine.add('MOVE_TO_CONVEYOR', move_base_safe("CB02"),
                transitions={'succeeded': 'OPEN_GRIPPER',
                            'failed': 'ARM_TO_CONVEYOR_VIEW'})

        smach.StateMachine.add('OPEN_GRIPPER',  gms_o.control_gripper('open_wide'),
            transitions={'succeeded': 'ARM_TO_CONVEYOR_VIEW'})

        smach.StateMachine.add('OPEN_NARROW',  gms_o.control_gripper('open'),
                    transitions={'succeeded': 'ARM_TO_CONVEYOR_VIEW'})

        smach.StateMachine.add('ARM_TO_CONVEYOR_VIEW', gms_o.move_arm(
          [2.20925454919, 1.98791323601 ,-1.38040010402 ,2.87320772063, 4.35358370112]),
          transitions={'succeeded': 'OPEN_GRIPPER_WIDE',
                       'failed': 'ARM_TO_CONVEYOR_VIEW'})

        smach.StateMachine.add('OPEN_GRIPPER_WIDE',  gms_o.control_gripper('open_wide'),
            transitions={'succeeded': 'WAIT_FOR_OBJECT'})

        smach.StateMachine.add('WAIT_FOR_OBJECT', wait_for_object(),
             transitions={'succeeded': 'WAIT_TO_CLOSE_GRIPPER',
                         'failed': 'SET_ACTION_LIB_FAILURE'})

        smach.StateMachine.add('WAIT_TO_CLOSE_GRIPPER', my_pause(),
            transitions={'succeeded': 'CBT_GRASP'})

        smach.StateMachine.add('CBT_GRASP', gms_o.control_gripper('close'),
            transitions={'succeeded': 'SET_ACTION_LIB_SUCCESS'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True),
                               transitions={'succeeded':'OVERALL_SUCCESS'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False),
                               transitions={'succeeded':'OVERALL_FAILED'})

    # smach viewer
    sis = smach_ros.IntrospectionServer('pick_object_rtt_smach_viewer', SM, '/PICK_OBJECT_RTT_SMACH_VIEWER')
    sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'pick_object_rtt_server',
        action_spec = PickObjectAction,
        wrapped_container = SM,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'pick_object_goal',
        feedback_key = 'pick_object_feedback',
        result_key   = 'pick_object_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) > 1:
       main(simulation=(sys.argv[1]=='mokeup'))
    else:
       main(simulation=False)
