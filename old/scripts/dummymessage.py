#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('/Transport', String, queue_size = 10)
	rospy.init_node('talker',anonymous=True)
	rate = rospy.Rate(10)
	time.sleep(10)
	x = 0
	while not rospy.is_shutdown():
		while x != 1:
			hello_str = "3141019043(5,100,100,200,200,300,300,400,400,500,500)1234567890123456789012345678901234567890123456789012345678901234"
			rospy.loginfo(hello_str)
                	pub.publish(hello_str)
                	rate.sleep()
			x = 1
		hello_str = "3141022015(1,120,130,314)1234567890123456789012345678901234567890123456789012345678901234" 
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass


