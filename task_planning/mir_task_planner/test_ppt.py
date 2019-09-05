import rospy
from refbox_communication import Refbox_Communication 
from robot_behaviours import Robot_Behaviours

if __name__ == '__main__':
    rospy.init_node('ppt')
    robot = Robot_Behaviours() 
    refbox = Refbox_Communication()
    #refbox.wait_for_refbox()

    for i in range(2):
        robot.move_base('WS02')
        robot.perceive(for_test='PPT')
        robot.pick_all()
        insert_failed_flag = False
        if 0 != robot.number_of_objects_picked():
            robot.move_base('PP01')
            robot.perceive_cavity()
            robot.place_in_cavity()
        if 0 != robot.number_of_objects_picked():
            robot.move_base('PP01')
            robot.perceive_cavity()
            robot.place_in_cavity()
        if 0 != robot.number_of_objects_picked():
            robot.move_base('PP01')
            robot.perceive_cavity()
            robot.place_in_cavity()
        
