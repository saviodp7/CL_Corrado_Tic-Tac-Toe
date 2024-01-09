#!/usr/bin/env python3

import rospy
from camera_msg.msg import Camera
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('chatter', Camera)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	camera_msg = Camera()
	camera_msg.header.stamp = rospy.Time.now()
	camera_msg.score = 8
	camera_msg.mossa= 3 
		
	while not rospy.is_shutdown():
		rospy.loginfo(camera_msg)
		pub.publish(camera_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
	   talker()
	except rospy.ROSInterruptException: pass
