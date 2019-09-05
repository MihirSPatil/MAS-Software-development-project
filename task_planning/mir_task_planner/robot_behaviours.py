#! /usr/bin/env python
import rospy
import roslib
import actionlib

import sys
import collections

from mir_yb_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from mir_yb_action_msgs.msg import PerceiveLocationAction, PerceiveLocationGoal
from mir_yb_action_msgs.msg import PickObjectWBCAction, PickObjectWBCGoal
from mir_yb_action_msgs.msg import StageObjectAction, StageObjectGoal
from mir_yb_action_msgs.msg import PlaceObjectAction, PlaceObjectGoal
from mir_yb_action_msgs.msg import UnStageObjectAction, UnStageObjectGoal
from mir_yb_action_msgs.msg import PerceiveLocationAction, PerceiveLocationGoal
from mir_yb_action_msgs.msg import InsertObjectAction, InsertObjectGoal

class Robot_Behaviours():

    def __init__(self):
        rospy.loginfo('initializing all the clients and waiting')
        self.move_base_client =  actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
        self.move_base_client.wait_for_server()
        self.perceive_location_client = actionlib.SimpleActionClient('perceive_location_server', PerceiveLocationAction)
        self.perceive_location_client.wait_for_server()
        self.pick_client = actionlib.SimpleActionClient('wbc_pick_object_server', PickObjectWBCAction) 
        self.pick_client.wait_for_server()
        self.stage_client = actionlib.SimpleActionClient('stage_object_server', StageObjectAction)
        self.stage_client.wait_for_server()
        self.unstage_client = actionlib.SimpleActionClient('unstage_object_server', UnStageObjectAction)
        self.unstage_client.wait_for_server()
        self.place_client = actionlib.SimpleActionClient('place_object_server', PlaceObjectAction)
        self.place_client.wait_for_server()
        self.cavity_perceive_client = actionlib.SimpleActionClient('perceive_cavity_server', PerceiveLocationAction)
        self.cavity_perceive_client.wait_for_server()
        self.insert_in_cavity_client = actionlib.SimpleActionClient('insert_object_in_cavity_server', InsertObjectAction)
        self.insert_in_cavity_client.wait_for_server()

        rospy.loginfo('all clients initialized READY !!!!!')

        #dictionary for platform occupied
        self.platform_occupied_dict = {'platform_middle':None,'platform_left':None,'platform_right':None}

        #Priority of objects. All the objects need to be mentioned 
        self.objects_priority = ['F20_20_B', 'F20_20_G', 'S40_40_B', 'S40_40_G',
                             'R20', 'BEARING_BOX', 'MOTOR', 'M30',
                             'AXIS', 'M20', 'BEARING', 'DISTANCE_TUBE', 
                             'M20_100', 'CONTAINER_BOX_BLUE', 'CONTAINER_BOX_RED']

        #Priority of objects. All the objects need to be mentioned 
        self.objects_priority_cavities = ['F20_20_B', 'F20_20_G', 'S40_40_B', 'S40_40_G',
                                          'R20', 'M30', 'M20', 'M20_100']


    def move_base(self, destination):
        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = 'barrier_tape'
        goal.source_location = ''
        goal.destination_location = destination
        timeout = 60.0
        rospy.loginfo('Sending action lib goal to move_base_safe_server, source : ' + goal.source_location + ' , destination : ' + goal.destination_location)
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        # cancel the goal
        self.move_base_client.cancel_goal()
        rospy.loginfo('Move base complete')
    
    def perceive(self, for_test='BMT'):
        goal = PerceiveLocationGoal()
        goal.location = ""
        self.perceive_location_client.send_goal(goal)
        self.perceive_location_client.wait_for_result(rospy.Duration.from_sec(15.0))
        self.perceive_location_client.cancel_goal()
        received_flag = False 
        while(not received_flag):
            result = self.perceive_location_client.get_result()
            if(result):
                received_flag = True
            rospy.sleep(0.1)

        self.perceived_objects = result.object_list
        rospy.loginfo('Perceived objects : ' + ','.join(self.perceived_objects))
        self.perceived_objects = self.prioritize_object_list(for_test)
        rospy.loginfo('Priority Perceived objects : ' + ','.join(self.perceived_objects))

    def prioritize_object_list(self, for_test='BMT'):
        # First count the number of repeated objects in perceived list
        # Loop through the priority list,
        # Find if the object exist in perceived list
        # Add to the prioritized_perceived list
        
        perceived_objects_in_caps = map(str.upper, self.perceived_objects)
        perceived_object_name_dict = dict(zip(perceived_objects_in_caps, self.perceived_objects))
        count_of_objects = collections.Counter(perceived_objects_in_caps)
        prioritized_perceived_objects = []
        if 'PPT' == for_test:
            priorities = self.objects_priority_cavities
        else:
            priorities = self.objects_priority

        for o in priorities: 
            if o.upper() in perceived_objects_in_caps:
                for count in range(count_of_objects[o]):
                    prioritized_perceived_objects.append(perceived_object_name_dict[o])

        return prioritized_perceived_objects

    def stage(self, object_name):
        goal = StageObjectGoal()
        platform = None
        #Find out which platform is not occupied
        for l, s in self.platform_occupied_dict.iteritems():
            if s == None:
                platform = l
                break

        if(platform):
            #Filling the dictionary with the value
            self.platform_occupied_dict[platform] = object_name

            # Now filing the goal
            goal.robot_platform = platform
            rospy.loginfo('Stage to : ' +  goal.robot_platform)
            self.stage_client.send_goal(goal)
            self.stage_client.wait_for_result(rospy.Duration.from_sec(15.0))
        else:
            rospy.logerror('No place to stage')

    def pick(self, object_name):
        goal = PickObjectWBCGoal()

        goal.object = object_name 
        rospy.loginfo('Sending action lib goal to pick_object_server : ' + goal.object)
        self.pick_client.send_goal(goal)
        self.pick_client.wait_for_result(rospy.Duration.from_sec(15.0))
        result = self.pick_client.get_result()
        while result is None:
            result = self.pick_client.get_result()
            rospy.sleep(0.1)

        rospy.loginfo('Result from Pick server ' + str(result))
        return result.success


    def pick_all(self):
        # picking all the objects
        for o in self.perceived_objects:
            if (self.pick(o)):
                #if pick passed then stage 
                self.stage(o)
            else :
                rospy.loginfo('Pick failed')

            #check if ther is any empty platform
            if None not in self.platform_occupied_dict.values():
                rospy.loginfo('No more place in backplatform ')
                break

    def unstage(self, platform):
        goal = UnStageObjectGoal()
        goal.robot_platform = platform 
        self.unstage_client.send_goal(goal)
        self.unstage_client.wait_for_result(rospy.Duration.from_sec(15.0))
        rospy.loginfo("Unstaged from backplatform " + platform)

        #Resetting the dictionary
        self.platform_occupied_dict[platform] = None

    def place(self, location_name):
        goal = PlaceObjectGoal()
        goal.object = ''
        goal.location = location_name
        timeout = 15.0
        rospy.loginfo('Sending action lib goal to place_object_server : ' + goal.object + ' ' + goal.location)
        self.place_client.send_goal(goal)
        self.place_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))

    def place_all(self, location_name):
        for l, o in self.platform_occupied_dict.iteritems():
            if o is not None:
                self.unstage(l)
                self.place(location_name)
                #Resetting  the platofrm dictionary with None
                self.platform_occupied_dict[l] = None

    def perceive_cavity(self):
        goal = PerceiveLocationGoal()
        goal.location = ""
        self.cavity_perceive_client.send_goal(goal)
        self.cavity_perceive_client.wait_for_result(rospy.Duration.from_sec(15.0))
        rospy.loginfo('Result from perceive cavity server ' + str(self.cavity_perceive_client.get_result()))
    
    def place_in_cavity(self, test_objects=None):
        if(test_objects):
            for l, o in test_objects.iteritems():
                if o is not None:
                    self.platform_occupied_dict[l] = o

        for l, o in self.platform_occupied_dict.iteritems():
            if o is not None:
                goal = InsertObjectGoal()
                goal.hole = "PP01"
                goal.peg = o
                goal.robot_platform = l
                timeout = 65.0
                rospy.loginfo('Sending action lib goal to insert_object_in_cavity_server : ' + goal.peg)
                self.insert_in_cavity_client.send_goal(goal)
                self.insert_in_cavity_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
                result = self.insert_in_cavity_client.get_result()
                print (result)
                if result and True == result.success:
                    #Resetting  the platofrm dictionary with None
                    self.platform_occupied_dict[l] = None


    def number_of_objects_picked(self):
        # Length of the dict - count of the None values
        return len(self.platform_occupied_dict) - self.platform_occupied_dict.values().count(None)
