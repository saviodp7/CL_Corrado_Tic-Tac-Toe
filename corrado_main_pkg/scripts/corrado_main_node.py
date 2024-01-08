#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from CorradoTrajectoryController import CorradoTrajectoryController

def main():
    rospy.init_node('corrado_main_node', anonymous=True)
    print("hello node!")

    corrado_controller = CorradoTrajectoryController()
    corrado_controller.homing()
    for i in range(9):
        corrado_controller.draw_circle(i)

    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass