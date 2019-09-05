import rospy
from refbox_communication import Refbox_Communication 
from robot_behaviours import Robot_Behaviours

if __name__ == '__main__':
    rospy.init_node('btt')
    robot = Robot_Behaviours() 
    refbox = Refbox_Communication()
    refbox.wait_for_refbox()

    for i in range(5):
        #check if move_base failed
        robot.move_base(refbox.pick_object_from())
        robot.perceive(for_test='BMT')
        robot.pick_all()
        robot.move_base(refbox.place_object_to())
        robot.place_all(refbox.place_object_to())

    rospy.spin()
