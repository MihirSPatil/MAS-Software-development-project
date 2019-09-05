#!/usr/bin/python

import rospy
import smach
import smach_ros

# import of generic states
import mir_states.common.navigation_states as gns
import mir_states.common.manipulation_states as gms
import mcr_states.common.perception_states as gps
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import PerceiveLocationAction
from mir_yb_action_msgs.msg import PerceiveLocationFeedback
from mir_yb_action_msgs.msg import PerceiveLocationResult

# perception object list
import mcr_perception_msgs.msg

#===============================================================================

class wait_for(smach.State):
    def __init__(self, sleep_time):
        smach.State.__init__(self, outcomes=['success'])
        self.sleep_time = sleep_time
    def execute(self, userdata):
        rospy.sleep(self.sleep_time)
        return 'success'

class getGoal(smach.State): # inherit from the State base class
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                   input_keys=['perceive_location_goal'],
                                   output_keys=['perceive_location_feedback', 'perceive_location_result'])
        self.counter = 0

    def execute(self, userdata):
        # updating result (false until finished all actions)
        result = PerceiveLocationResult()
        result.success = False
        userdata.perceive_location_result = result
        # get arm goal from actionlib
        platform = userdata.perceive_location_goal.location
        # giving feedback to the user
        feedback = PerceiveLocationFeedback()
        feedback.current_state = 'GET_GOAL'
        feedback.text='[perceive_location] Perceiving location : ' + platform
        userdata.perceive_location_feedback = feedback
        return 'succeeded'

#===============================================================================

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['perceive_location_goal'],
                                    output_keys=['perceive_location_feedback', 'perceive_location_result'])
        self.result = result
        self.objects_sub = rospy.Subscriber('/mcr_perception/object_list_merger/output_object_list', mcr_perception_msgs.msg.ObjectList, self.objects_callback)
        self.merged_object_list = []

    def execute(self, userdata):
        result = PerceiveLocationResult()
        result.success = self.result
        userdata.perceive_location_result = result
        for o in self.merged_object_list:
            result.object_list.append(o)

        self.merged_object_list = []
        return 'succeeded'

    def objects_callback(self, msg):
        for o in msg.objects:
            self.merged_object_list.append(str(o.name))
#===============================================================================

def main():
    rospy.init_node('perceive_location_server')
    sleep_time = rospy.get_param('~sleep_time', 1.0)
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['perceive_location_goal'],
            output_keys = ['perceive_location_feedback', 'perceive_location_result'])
    # Open the container
    with sm:
        # approach to platform
        smach.StateMachine.add('GET_GOAL', getGoal(),
            transitions={'succeeded':'PUBLISH_REFERENCE_FRAME'})
            #transitions={'succeeded':'START_OBJECT_LIST_MERGER'})

        # generates a pose based on the previous string object topic received
        smach.StateMachine.add('PUBLISH_REFERENCE_FRAME', gbs.send_event([('/static_transform_publisher_node/event_in', 'e_start')]),
                transitions={'success':'SUBSCRIBE_TO_POINT_CLOUD'})

        smach.StateMachine.add('SUBSCRIBE_TO_POINT_CLOUD', gbs.send_event([('/mcr_perception/mux_pointcloud/select', '/arm_cam3d/depth_registered/points')]),
                transitions={'success':'START_OBJECT_LIST_MERGER'})

        smach.StateMachine.add('START_OBJECT_LIST_MERGER', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/event_in', 'e_start'),
                               ('/mcr_perception/object_selector/event_in', 'e_start')],
                event_out_list=[('/mcr_perception/object_list_merger/event_out', 'e_started', True)],
                timeout_duration=5),
                transitions={'success': 'LOOK_AT_WORKSPACE',
                             'timeout': 'SET_ACTION_LIB_FAILURE',
                             'failure': 'SET_ACTION_LIB_FAILURE'})

        # send arm to a position in which the depth camera can see the platformsmach.StateMachine.add('LOOK_AT_WORKSPACE_LEFT', gms.move_arm('look_at_workspace_LEFT'),
        smach.StateMachine.add('LOOK_AT_WORKSPACE', gms.move_arm('look_at_workspace_from_far'),
            transitions={'succeeded': 'START_SEGMENTATION',
                            'failed': 'LOOK_AT_WORKSPACE'})

         # this is new scene segmentation pipeline combined with object detection
        smach.StateMachine.add('START_SEGMENTATION', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/scene_segmentation/event_in', 'e_start')],
                event_out_list=[('/mcr_perception/scene_segmentation/event_out', 'e_started', True)],
                timeout_duration=5),
                transitions={'success': 'WAIT_LEFT',
                             'timeout': 'SET_ACTION_LIB_FAILURE',
                             'failure': 'SET_ACTION_LIB_FAILURE'})

        smach.StateMachine.add('WAIT_LEFT', wait_for(sleep_time=sleep_time),
            transitions={'success':'ADD_POINT_CLOUD'})

        smach.StateMachine.add('ADD_POINT_CLOUD', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/scene_segmentation/event_in', 'e_add_cloud_start')],
                event_out_list=[('/mcr_perception/scene_segmentation/event_out', 'e_add_cloud_stopped', True)],
                timeout_duration=5),
                transitions={'success': 'RECOGNIZE_OBJECTS',
                             'timeout': 'ADD_POINT_CLOUD',
                             'failure': 'SET_ACTION_LIB_FAILURE'})
        
        # in the new scene_segmentation pipeline, object detector can be called by string msg e_segment
        smach.StateMachine.add('RECOGNIZE_OBJECTS', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/multimodal_object_identification/input/event_in', 'e_trigger'), 
                               ('/mcr_perception/scene_segmentation/event_in', 'e_segment')],
                event_out_list=[('/mcr_perception/scene_segmentation/event_out', 'e_done', True),
                                ('/mcr_perception/multimodal_object_identification/output/event_out', 'e_done', True)],
                timeout_duration=7),
                transitions={'success': 'STOP_PUBLISH_ACCUMULATED_PC',
                             'timeout': 'STOP_PUBLISH_ACCUMULATED_PC',
                             'failure': 'STOP_PUBLISH_ACCUMULATED_PC'})


        smach.StateMachine.add('STOP_PUBLISH_ACCUMULATED_PC', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/scene_segmentation/event_in', 'e_stop')],
                event_out_list=[('/mcr_perception/scene_segmentation/event_out', 'e_stopped', True)],
                timeout_duration=5),
                transitions={'success': 'STOP_COMPONENTS',
                             'timeout': 'STOP_COMPONENTS',
                             'failure': 'STOP_COMPONENTS'})

        smach.StateMachine.add('STOP_COMPONENTS', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/event_in', 'e_stop'), ('/mcr_perception/scene_segmentation/event_in', 'e_stop')],
                event_out_list=[('/mcr_perception/object_list_merger/event_out', 'e_stopped', True), ('/mcr_perception/scene_segmentation/event_out', 'e_stopped', True)],
                timeout_duration=5),
                transitions={'success': 'UNSUBSCRIBE_FROM_POINT_CLOUD',
                             'timeout': 'UNSUBSCRIBE_FROM_POINT_CLOUD',
                             'failure': 'UNSUBSCRIBE_FROM_POINT_CLOUD'})

        smach.StateMachine.add('UNSUBSCRIBE_FROM_POINT_CLOUD', gbs.send_event([('/mcr_perception/mux_pointcloud/select', '/empty_topic')]),
                transitions={'success':'PUBLISH_MERGED_OBJECT_LIST'})

        smach.StateMachine.add('PUBLISH_MERGED_OBJECT_LIST', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/event_in', 'e_trigger')],
                event_out_list=[('/mcr_perception/object_list_merger/event_out', 'e_done', True)],
                timeout_duration=5),
                transitions={'success': 'SET_ACTION_LIB_SUCCESS',
                             'timeout': 'SET_ACTION_LIB_FAILURE',
                             'failure': 'SET_ACTION_LIB_SUCCESS'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True),
                               transitions={'succeeded':'OVERALL_SUCCESS'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False),
                               transitions={'succeeded':'OVERALL_FAILED'})
    # smach viewer
    sis = smach_ros.IntrospectionServer('perceive_location_smach_viewer', sm, '/PERCEIVE_LOCATION_SMACH_VIEWER')
    sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'perceive_location_server',
        action_spec = PerceiveLocationAction,
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'perceive_location_goal',
        feedback_key = 'perceive_location_feedback',
        result_key   = 'perceive_location_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()
