#!/usr/bin/env python

import rospy
from dbw_mkz_msgs.msg import TwistCmd
import random

def speed_display_tester():
	pub = rospy.Publisher('test_twistcmd', TwistCmd, queue_size=10)
	rospy.init_node('speed_display_twistcmd_tester', anonymous = False)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		msg = TwistCmd()
		f = random.uniform(60, 70)
		msg.twist.linear.x = f
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

if __name__ == "__main__":
	try:
		speed_display_tester()
	except rospy.ROSInterruptException:
		pass