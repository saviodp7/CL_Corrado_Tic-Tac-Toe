#!/usr/bin/env python3

import rospy
#from std_msgs.msg import String
#from CorradoTrajectoryController import CorradoTrajectoryController
#from tris_main import tris_main
from corrado_camera.msg import BestMove
#from corrado_camera_node import corrado_camera_node

mossa_ricevuto=0
score_ricevuto=0

def callback(best_move_msg):
    global score_ricevuto
    global mossa_ricevuto
    score_ricevuto=best_move_msg.score
    mossa_ricevuto=best_move_msg.mossa
    rospy.loginfo("Messaggio ricevuto: %s", best_move_msg.mossa)

def main():
    rospy.init_node("game_node", anonymous=True)

    mossa_corrado_pub=rospy.Publisher('mossa', int, queue_size=1)
    mossa_corrado_sub=rospy.Subscriber('best_move', BestMove, callback)

    rate=rospy.Rate(5)

    turno=False
    while not rospy.is_shutdown(): 

        input("Posso iniziale? Premi invio...")
        turno = True
        if turno==True:
            #salvo la mosssa
            #salvo lo score
            #pubblico mossa e score sul nodo robot e a video
            msg_mossa=mossa_ricevuto
            mossa_corrado_pub.publish(msg_mossa)
            

            input("Ora tocca a te")
    
    rospy.spinOnce()
    
    rate.sleep()









