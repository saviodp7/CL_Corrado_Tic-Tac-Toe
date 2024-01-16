#!/usr/bin/env python3
import rospy

import time 
from corrado_camera.msg import BestMove
from std_msgs.msg import Int8

mossa = None

def callback(best_move_msg):
    global mossa
    mossa = best_move_msg.mossa
    rospy.loginfo(f"Mossa migliore: {best_move_msg.mossa}, score: {best_move_msg.score}")

def main():
    rospy.init_node("game_node", anonymous=True)
    rate = rospy.Rate(0.5) #Hz

    mossa_corrado_pub = rospy.Publisher('cmd_move', Int8, queue_size=1)
    rospy.Subscriber('best_move', BestMove, callback)

    time.sleep(30)

    while not rospy.is_shutdown(): 

        input("Premi invio per eseguire la mossa visualizzata...\n")
        if mossa != None:
            mossa_corrado_pub.publish(mossa)
        else:
            rospy.logerr("NESSUNA MOSSA VALIDA TROVATA!!!")
    
    rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass







