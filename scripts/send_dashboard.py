#!/usr/bin/env python

import rospy
from octopus_rviz_plugin.msg import dashboard
import random

def newValue(oldV, newV):
	alpha = 0.02
	return newV * alpha + oldV * (1-alpha);

def dashboard_display_tester():
	pub = rospy.Publisher('test_dashboard', dashboard, queue_size=10)
	rospy.init_node('dashboard_tester', anonymous = False)
	rate = rospy.Rate(10)

	msg = dashboard()
	count = 0
	while not rospy.is_shutdown():
		count += 1
		
		msg.speed = newValue(msg.speed, random.uniform(30, 120) ) 
		msg.throttle_percent = newValue(msg.throttle_percent, random.uniform(0.2, 0.8) )
		msg.brake_percent = newValue(msg.brake_percent, random.uniform(0.1, 0.4) )
		msg.steering_wheel_angle = newValue(msg.steering_wheel_angle, random.uniform(-1, 1) )
		if (count % 20 == 0):
			msg.turn_signal = random.randint(0,2)

		msg.x_accel_data = newValue(msg.x_accel_data, random.uniform(20, 50) )
		msg.x_accel_upper_bound = 100
		msg.x_accel_lower_bound = -100
		msg.x_accel_upper_comfort = 80
		msg.x_accel_lower_comfort = -80

		msg.x_jerk_data = newValue(msg.x_jerk_data, random.uniform(-5, 5) )
		msg.x_jerk_upper_bound = 10
		msg.x_jerk_lower_bound = -10
		msg.x_jerk_upper_comfort = 9
		msg.x_jerk_lower_comfort = -5

		

		msg.y_accel_data = newValue(msg.y_accel_data, random.uniform(-2.9, 0.9) )
		msg.y_accel_upper_bound = 1
		msg.y_accel_lower_bound = -1
		msg.y_accel_upper_comfort = 0.7
		msg.y_accel_lower_comfort = -0.3

		msg.y_jerk_data = newValue(msg.y_jerk_data, random.uniform(-0.9, 5) )
		msg.y_jerk_upper_bound = 2
		msg.y_jerk_lower_bound = -2
		msg.y_jerk_upper_comfort = 0.5
		msg.y_jerk_lower_comfort = -1.7


		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()


if __name__ == "__main__":
	try:
		dashboard_display_tester()
	except rospy.ROSInterruptException:
		pass