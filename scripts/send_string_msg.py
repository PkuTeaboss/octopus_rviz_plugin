#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import random

def speed_display_tester():
	pub = rospy.Publisher('test_speed_limit', String, queue_size=10)
	rospy.init_node('speed_limit_display_tester', anonymous = False)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		f = random.randint(10, 90)
		rospy.loginfo(f)
		pub.publish(str(f))
		rate.sleep()

if __name__ == "__main__":
	try:
		speed_display_tester()
	except rospy.ROSInterruptException:
		pass