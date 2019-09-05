#!/usr/bin/python

import rospy
import smach
import smach_ros

import tf
from geometry_msgs.msg import PoseStamped

import mir_states.common.navigation_states as gns
import mir_states.common.manipulation_states as gms
import mcr_states.common.perception_states as gps
import mcr_states.common.basic_states as gbs

#===================================base controller ============================================
class ShiftBaseCommand(smach.State): # inherit from the State base class
    TOPIC_OUT = '/mcr_perception/percieve_location_server/pose_transformer/pose_in'

    def __init__(self, base_shift = [0.0, 0.0, 0.0]):
        smach.State.__init__(self, outcomes=['succeeded'], 
                                   input_keys=[], 
                                   output_keys=[])
        self.base_shift = base_shift
        self.base_rot = [0.0 ,0.0,0.0,1.0] #list(tf.transformations.quaternion_from_euler(0, 0, base_shift[2]))
        self.pub = rospy.Publisher(ShiftBaseCommand.TOPIC_OUT, PoseStamped, queue_size=1)

    def execute(self, userdata):
        # updating result (false until finished all actions)

        base_shift_pose = PoseStamped()
        base_shift_pose.header.frame_id = 'base_link'
        base_shift_pose.pose.position.x = self.base_shift[0]
        base_shift_pose.pose.position.y = self.base_shift[1]

        base_shift_pose.pose.orientation.x = self.base_rot[0]
        base_shift_pose.pose.orientation.y = self.base_rot[1]
        base_shift_pose.pose.orientation.z = self.base_rot[2]
        base_shift_pose.pose.orientation.w = self.base_rot[3]

        self.pub.publish(base_shift_pose)

        return 'succeeded'

################################################################
class ShiftBase(smach.StateMachine):
    def __init__(self, base_shift = [0.0, 0.0, 0.0]):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'timeout'],
                                          input_keys=[],
                                          output_keys=[])

        with self:

            # get pose from action lib as string, convert to pose stamped and publish
            smach.StateMachine.add('SHIFT_BASE_COMMAND', ShiftBaseCommand(base_shift),
                    transitions={'succeeded': 'MOVE_BASE'})
            
            # send event_in to move base to a pose
            smach.StateMachine.add('MOVE_BASE', gbs.send_and_wait_events_combined(
                    event_in_list=[('/mcr_perception/percieve_location_server/pose_transformer/event_in','e_start'), ],
                    event_out_list=[('/mcr_perception/percieve_location_server/pose_transformer/event_out','e_success', True)],
                    timeout_duration=50),
                    transitions={'success':'ADJUST_BASE',
                                 'timeout':'timeout',
                                 'failure':'failed'})
            
            # call direct base controller to fine adjust the base to the final desired pose 
            # (navigation tolerance is set to a wide tolerance)
            smach.StateMachine.add('ADJUST_BASE', gbs.send_and_wait_events_combined(
                    event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_start')],
                    event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_success', True)],
                    timeout_duration=5), # this is a tradeoff between speed and accuracy, set a higher value for accuracy increase
                    transitions={'success':'STOP_CONTROLLER_WITH_SUCCESS',
                                 'timeout':'STOP_CONTROLLER_WITH_SUCCESS',
                                 'failure':'STOP_CONTROLLER_WITH_FAILURE'})
            
            # stop controller with success
            smach.StateMachine.add('STOP_CONTROLLER_WITH_SUCCESS', gbs.send_and_wait_events_combined(
                    event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_stop')],
                    event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_stopped', True)],
                    timeout_duration=1),
                    transitions={'success':'succeeded',
                                 'timeout':'succeeded',
                                 'failure':'failed'})
                                 
            # stop controller with failure
            smach.StateMachine.add('STOP_CONTROLLER_WITH_FAILURE', gbs.send_and_wait_events_combined(
                    event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_stop')],
                    event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_stopped', True)],
                    timeout_duration=1),
                    transitions={'success':'failed',
                                 'timeout':'failed',
                                 'failure':'failed'})


################################################################
class Accumulate_Point_Cloud(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'timeout'],
                                          input_keys=[],
                                          output_keys=[])

        with self:
            # Move robot
            #smach.StateMachine.add('SHIFT_BASE_1', ShiftBase([0.0, 0.05, 0.0]),
            #transitions={'succeeded': 'failed',
            #             'timeout': 'failed',
            #             'failed': 'failed'})

            # start point could accumulator
            smach.StateMachine.add('START_ACCUMULATOR', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/cloud_accumulator/event_in', 'e_start')],
                event_out_list=[('/mcr_perception/cloud_accumulator/event_out', 'e_started', True)],
                timeout_duration=5),
                transitions={'success': 'ADD_POINT_COULD_1',
                             'timeout': 'timeout',
                             'failure': 'failed'})

            # add point could
            smach.StateMachine.add('ADD_POINT_COULD_1', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/cloud_accumulator/event_in', 'e_add_cloud_start')],
                event_out_list=[('/mcr_perception/cloud_accumulator/event_out', 'e_add_cloud_stopped', True)],
                timeout_duration=5),
                transitions={'success': 'SHIFT_BASE',
                             'timeout': 'ADD_POINT_COULD_1',
                             'failure': 'failed'})
            
            # Move robot
            smach.StateMachine.add('SHIFT_BASE', ShiftBase([0.0, 0.30, 0.0]),
            transitions={'succeeded': 'ADD_POINT_COULD_2',
                         'timeout': 'ADD_POINT_COULD_2',
                         'failed': 'ADD_POINT_COULD_2'})

            # add point could
            smach.StateMachine.add('ADD_POINT_COULD_2', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/cloud_accumulator/event_in', 'e_add_cloud_start')],
                event_out_list=[('/mcr_perception/cloud_accumulator/event_out', 'e_add_cloud_stopped', True)],
                timeout_duration=5),
                transitions={'success': 'SHIFT_BASE_BACK',
                             'timeout': 'ADD_POINT_COULD_2',
                             'failure': 'failed'})

            # Move robot
            smach.StateMachine.add('SHIFT_BASE_BACK', ShiftBase([0.0, -0.30, 0.0]),
            transitions={'succeeded': 'succeeded',
                         'timeout': 'succeeded',
                         'failed': 'succeeded'})
