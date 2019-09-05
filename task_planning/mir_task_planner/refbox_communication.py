#! /usr/bin/env python
import rospy
import atwork_ros_msgs.msg

class Refbox_Communication():
    def __init__(self):
        #Enum of object types from messages rocking
        #https://github.com/rockin-robot-challenge/at_work_central_factory_hub/blob/rockin/rockin/msgs/Inventory.proto
        self.object_type = {1:'F20_20_B', 2:'F20_20_G', 3:'S40_40_B',
                            4:'S40_40_G' , 5:'M20_100', 6:'M20',
                            7:'M30' , 8:'R20', 9:'BEARING_BOX',
                            10:'BEARING' , 11:'AXIS', 12:'DISTANCE_TUBE',
                            13:'MOTOR', 14:'CONTAINER_BOX_BLUE', 15:'CONTAINER_BOX_RED'}

        self.location_type = {1:'SH', 2:'WS', 3:'CB', 4:'WP', 5:'PP', 6:'ROBOT'}

        #Subscriber -> subscribe to refbox for Order message
        rospy.Subscriber("/robot_example_ros/task_info", atwork_ros_msgs.msg.TaskInfo, self.task_callback, queue_size=10)

        self.wait_for_task_flag = True
        self.source_location = None
        self.destination_location = None

    def task_callback(self, msg):
        self._tasks = atwork_ros_msgs.msg.TaskInfo(msg.tasks)
        for task in self._tasks.tasks:
            #self.object_type[task.transportation_task.object.type.data]
            self.source_location =  self.location_type[task.transportation_task.source.type.data] + \
                                              str(task.transportation_task.source.instance_id.data ).zfill(2)
            self.destination_location = self.location_type[task.transportation_task.destination.type.data] + \
                                              str(task.transportation_task.destination.instance_id.data ).zfill(2)
            break;
        self.wait_for_task_flag = False


    def wait_for_refbox(self):
        while(self.wait_for_task_flag):
            rospy.loginfo("Waiting for commands from refbox !")
            rospy.sleep(2)

    def pick_object_from(self):
        return self.source_location

    def place_object_to(self):
        return self.destination_location
