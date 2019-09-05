#!/usr/bin/env python

import rospy
import actionlib
import smach
import smach_ros
from std_msgs.msg import String
import std_msgs.msg
import os

#import atwork_ros_msgs.msg
from mir_plan_loader_msgs.msg import LoadPlanAction, LoadPlanGoal
from mir_planner_executor_msgs.msg import ExecutePlanAction, ExecutePlanGoal
from mir_planner_executor_msgs.srv import ReAddGoals

# for send and wait combined
# import mir_states.common.basic_states as gbs

# for smach to respond to ctrl + c interruption, create another thread
# and call for preemption from there
import threading
#===============================================================================

class re_add_goals(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['success', 'failure'])
        rospy.wait_for_service('/task_planning/re_add_goals')
        self.srv = rospy.ServiceProxy('/task_planning/re_add_goals', ReAddGoals)
        rospy.sleep(0.2)

    def execute(self, userdata):
        res = self.srv()
        if res:
            return 'success'
        else:
            return 'failure'

class execute_plan(smach.State):
    def __init__(self, filename):
        smach.State.__init__(self,  outcomes=['success', 'failure', 'preempted'])
        self.filename = filename;
        self.plan_loader_client = actionlib.SimpleActionClient('/task_planning/load_plan', LoadPlanAction)
        self.plan_loader_client.wait_for_server()
        self.planner_executor_client = actionlib.SimpleActionClient('/task_planning/execute_plan', ExecutePlanAction)
        self.planner_executor_client.wait_for_server()
        rospy.sleep(0.2)

    def execute(self, userdata):
        suc, plan = self.load_plan()
        if not suc:
            return 'failure'
        suc = self.execute_plan(plan)
        if not suc:
            return 'failure'
        return 'success'

    def execute_plan(self, plan):
        goal = ExecutePlanGoal()
        goal.plan = plan
        self.planner_executor_client.send_goal(goal)
        finished = self.planner_executor_client.wait_for_result()
        if not finished:
            return (False)
        res =  self.planner_executor_client.get_result()
        return res.success

    def load_plan(self):
        goal = LoadPlanGoal()
        goal.file = self.filename
        # Fill in the goal here
        self.plan_loader_client.send_goal(goal)
        finished = self.plan_loader_client.wait_for_result(rospy.Duration.from_sec(5.0))
        if not finished:
            return (False, None)
        res = self.plan_loader_client.get_result()
        return (res.success, res.plan)

class send_event(smach.State):
    '''
    This class publishes "event" string on topic_name argument
    '''
    def __init__(self, topic_name, event):
        smach.State.__init__(self,  outcomes=['success'])
        # create publisher
        self.topic_name = topic_name
        self.event = event
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=1)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.2)

    def execute(self, userdata):
        # creating string message
        msg = String()
        # filling message
        msg.data = self.event
        # publish
        self.publisher.publish(msg)
        rospy.loginfo('publishing on ' + self.topic_name + ' ' + self.event)
        # weird error when cancelling ctrl + c the state machine it does not die
        #and prints a lot of stuff that gets fixed with this delay
        rospy.sleep(0.2)
        return 'success'

#===============================================================================

class wait_for_event(smach.State):
    '''
    This state will take a event name as input and waits for the event to
    be published.
    '''
    def __init__(self, topic_name, state_waiting_time=3., response_timeout=0.5, custom_event_msg=[]):
        """
        todo!
        state_waiting_time : the time which the state will wait for the event to be of positive outcome
        response_timeout : the expected time in which the node should repond to the request sent on event_in with a event_out
        """
        smach.State.__init__(self,  outcomes=['success', 'failure', 'timeout', 'preempted'])
        rospy.Subscriber(topic_name, String, self.event_cb)
        self.callback_msg_ = None
        self.message_received = False
        self.response_timeout = rospy.Duration.from_sec(response_timeout)
        self.state_waiting_time = state_waiting_time
        self.expected_msg = custom_event_msg
        self.known_msgs = ['e_success','e_failure']
        if custom_event_msg != []:
            for each in custom_event_msg:
                self.known_msgs.append(each)

    def msg_is_known(self):
        for each in self.known_msgs:
            if self.callback_msg_.data == each:
                # if the msg is known then return true
                return True
        # received msg was compared with all known msgs and did not found a match
        self.callback_msg_ = None
        return False

    def event_cb(self, callback_msg):
        self.callback_msg_ = callback_msg
        if self.msg_is_known():
            self.message_received = True
        else:
            # msg not known
            rospy.logerr('[wait for event] error : response message is not known')

    def sleep_while_checking_preemption(self):
        rospy.sleep(self.state_waiting_time)
        #self.state_waiting_time

    def getResult(self):
        """
        self.expected_msg = list of 2 strings
        self.expected_msg[0] = positive outcome that will return success
        self.expected_msg[1] = negative outcome that will return failure
        """
        # timeout
        if self.callback_msg_ is None:
            rospy.logwarn('event out message not received in the specified time')
            rospy.logwarn('Is the node running? if you are testing with terminal or rqt_node_event then dont worry about this message')
            return 'timeout'
        # if custom msg is empty we check for default return output
        if self.expected_msg == []:
            if self.callback_msg_.data == 'e_failure':
                self.callback_msg_ = None
                return 'failure'
            elif self.callback_msg_.data == 'e_success':
                self.callback_msg_ = None
                return 'success'
        # compare against custom msg
        # check for list to consist of 2 elements exactly
        if len(self.expected_msg) != 2:
            rospy.logerr('self.expected_msg can only be of 2 string outcomes, [0] positive, [1] negative')
            rospy.logwarn('Preempting state machine, fix implementation errors then run again !!')
            return 'preempted'
        if self.callback_msg_.data == self.expected_msg[0]:
            return 'success'
        elif self.callback_msg_.data == self.expected_msg[1]:
            # wait for some time, i.e. until new goals are available
            rospy.loginfo('Received negative outcome response, this is normal behavior, just waiting for some time')
            self.sleep_while_checking_preemption()
            return 'failure'
        # this code should never get executed
        rospy.logerr('[wait for event] error : This should not ever happen, if it does then there is a bug in the code')
        return 'failure'

    def execute(self, userdata):
        # Check for preempt
        if self.preempt_requested():
            rospy.logwarn('preemption requested!!!')
            return 'preempted'
        # reset flag of received to false
        self.message_received = False
        start_time = rospy.Time.now()
        rate = rospy.Rate(10) # 10hz
        # wait for message to arrive
        # TODO !! under test
        #self.callback_msg_ = None
        while((rospy.Time.now() - start_time < self.response_timeout) and not(self.message_received) and not(rospy.is_shutdown())):
            rate.sleep()
        # test, if you tested for a long time and this is still commented, then remove
        # self.message_received = False
        return self.getResult()

#---------------------------------------------------------------------

def main():
    rospy.init_node('planning_coordinator')
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['OVERALL_FAILURE', 'OVERALL_PREEMPT'])
    # Open the container
    with sm:
        # upload intrinsic knowledge
        smach.StateMachine.add('UPLOAD_INTRINSIC_KNOWLEDGE', send_event('/upload_knowledge/event_in', 'e_trigger'),
                               transitions={'success':'LOOK_FOR_UNFINISHED_GOALS'})

        # wait for upload intrinsic knowledge succes response TODO!! at the moment this is open loop

        #refbox communication
        #smach.StateMachine.add('GET_TASK', refbox.get_task(),
        #    transitions={'task_received':'TRIGGER_REFBOX',
        #                 'wrong_task':'GET_TASK',
        #                 'wrong_task_format':'GET_TASK'})

        #smach.StateMachine.add('TRIGGER_REFBOX', send_event('/refbox_parser/event_in', 'e_trigger'),
        #                       transitions={'success':'LOOK_FOR_UNFINISHED_GOALS'})

        # send even in to look for goals component to process and see if there are unfinished goals on the knowledge base
        smach.StateMachine.add('LOOK_FOR_UNFINISHED_GOALS', send_event('/knowledge_base_analyzer/pending_goals/event_in', 'e_start'),
                               transitions={'success':'WAIT_FOR_LOOK_FOR_UNFINISHED_GOALS'})#WAIT_FOR_LOOK_FOR_UNFINISHED_GOALS

        # wait for new goals query response
        smach.StateMachine.add('WAIT_FOR_LOOK_FOR_UNFINISHED_GOALS',
            wait_for_event('/knowledge_base_analyzer/pending_goals/event_out', custom_event_msg=['e_goals_available', 'e_no_goals']),
            transitions={'success':'GENERATE_PDDL_PROBLEM',
                        'failure':'RE_ADD_GOALS',
                        'timeout':'RE_ADD_GOALS',
                        'preempted':'OVERALL_PREEMPT'})

        smach.StateMachine.add('RE_ADD_GOALS', re_add_goals(),
                               transitions={'success':'LOOK_FOR_UNFINISHED_GOALS',
                                            'failure':'LOOK_FOR_UNFINISHED_GOALS'})

        # generate problem.pddl from knowledge base snapshot
        smach.StateMachine.add('GENERATE_PDDL_PROBLEM', send_event('/task_planning/pddl_problem_generator_node/event_in', 'e_trigger'),
                               transitions={'success':'WAIT_FOR_GENERATE_PDDL_PROBLEM'})

        # wait for pddl problem creator output
        smach.StateMachine.add('WAIT_FOR_GENERATE_PDDL_PROBLEM',
            wait_for_event('/task_planning/pddl_problem_generator_node/event_out'),
            transitions={'success':'MAKE_PLAN',#MAKE_PLAN
                        'failure':'OVERALL_FAILURE',
                        'timeout':'GENERATE_PDDL_PROBLEM',
                        'preempted':'OVERALL_PREEMPT'})

        # make plan
        smach.StateMachine.add('MAKE_PLAN', send_event('/mcr_task_planning/mercury_planner/event_in', 'e_trigger'),
                               transitions={'success':'WAIT_FOR_MAKE_PLAN'})

        # wait for planner to solve the planning problem
        smach.StateMachine.add('WAIT_FOR_MAKE_PLAN',
            wait_for_event('/mcr_task_planning/mercury_planner/event_out', response_timeout=5., custom_event_msg=['e_success','e_failure']),
            transitions={'success':'ANALYZE_PLAN_SUCCESS',
                        'failure':'OVERALL_PREEMPT',
                        'timeout':'WAIT_FOR_MAKE_PLAN',
                        'preempted':'OVERALL_PREEMPT'})

        # analize plan success
        smach.StateMachine.add('ANALYZE_PLAN_SUCCESS', send_event('/task_planning/plan_success_analizer/event_in', 'e_start'),
                               transitions={'success':'WAIT_FOR_ANALYZE_PLAN_SUCCESS'})#WAIT_FOR_ANALIZE_PLAN_SUCCESS

        # wait for new goals query response
        smach.StateMachine.add('WAIT_FOR_ANALYZE_PLAN_SUCCESS',
            wait_for_event('/task_planning/plan_success_analizer/event_out'),
            transitions={'success':'EXECUTE_PLAN',
                        'failure':'LOOK_FOR_UNFINISHED_GOALS',
                        'timeout':'ANALYZE_PLAN_SUCCESS',
                        'preempted':'OVERALL_PREEMPT'})

        smach.StateMachine.add('EXECUTE_PLAN', execute_plan(os.getenv("HOME") + '/.ros/mercury.plan'),
                               transitions={'success':'LOOK_FOR_UNFINISHED_GOALS',
                                            'failure':'LOOK_FOR_UNFINISHED_GOALS',
                                            'preempted':'OVERALL_PREEMPT'})

    # smach viewer
    sis = smach_ros.IntrospectionServer('planning_coordinator_viewer', sm, '/PLANNING_COORDINATOR_SM')
    sis.start()

    # Execute SMACH plan
    #outcome = sm.execute()

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    rospy.logwarn("ctrl + c detected!!! preempting smach execution")

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()


if __name__ == '__main__':
    main()
