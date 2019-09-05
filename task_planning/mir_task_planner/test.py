import rospy
from refbox_communication import Refbox_Communication 
from robot_behaviours import Robot_Behaviours

if __name__ == '__main__':
    rospy.init_node('ppt')
    robot = Robot_Behaviours() 
    #refbox = Refbox_Communication()
    #refbox.wait_for_refbox()

    #robot.move_base('WS01')
    #robot.perceive()
    #robot.pick_all()
    #robot.move_base('SH02')
    #robot.place_all('SH02')
    #if 0 != robot.number_of_objects_picked():
    #    robot.move_base('PP01')
    #dictionary for platform occupied
    platform_occupied_dict = {'platform_middle':'F20_20_B-00','platform_left':None,'platform_right':None}
    #robot.move_base('PP01')
    robot.perceive_cavity()
    robot.place_in_cavity(test_objects=platform_occupied_dict)
