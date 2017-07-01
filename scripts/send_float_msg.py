#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import random

def speed_display_tester():
	pub = rospy.Publisher('test_float32', Float32, queue_size=10)
	rospy.init_node('speed_display_tester', anonymous = False)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		f = random.uniform(60, 70)
		rospy.loginfo(f)
		pub.publish(f)
		rate.sleep()

if __name__ == "__main__":
	try:
		speed_display_tester()
	except rospy.ROSInterruptException:
		pass