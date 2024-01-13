#!/usr/bin/env python3

import rospy
from CorradoTrajectoryController import CorradoTrajectoryController
from std_msgs.msg import Int8

corrado_controller = CorradoTrajectoryController()

def callback(cmd_move):
    global corrado_controller
    if cmd_move.data == 12:
        corrado_controller.calib_traj()
    else:
        corrado_controller.draw_x(cmd_move.data)
        rospy.loginfo(f"\nEseguo la mossa {cmd_move}")

def main():
    rospy.init_node('corrado_main_node', anonymous=True)
    rospy.Subscriber('cmd_move', Int8, callback)

    corrado_controller.homing()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
