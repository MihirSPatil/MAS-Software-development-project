import rospy
from refbox_communication import Refbox_Communication 
from robot_behaviours import Robot_Behaviours

if __name__ == '__main__':
    rospy.init_node('ppt')
    robot = Robot_Behaviours() 
    refbox = Refbox_Communication()
    refbox.wait_for_refbox()

    for i in range(5):
        robot.move_base(refbox.pick_object_from())
        robot.perceive(for_test='PPT')
        robot.pick_all()
        if 0 != robot.number_of_objects_picked():
            robot.move_base('PP01')
            robot.perceive_cavity()
            robot.place_in_cavity()
