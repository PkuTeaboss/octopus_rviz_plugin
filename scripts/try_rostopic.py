#!/usr/bin/env python

import rospy
import rostopic
import inspect

def printObject(data, prespace = 0):
	try:
		data.__slots__
		print type(data)
		for key in data.__slots__:
			print key, '~'
			printObject(data.__getattribute__(key), prespace + 4)
	except AttributeError:
		print '-' * prespace, data



def callback(data):
	#rospy.loginfo(data)
	#rospy.loginfo(type(data))
	printObject(data)



def try_topic():
	topic = '/test_twistcmd'
	msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic)
	print msg_class, real_topic, msg_eval
	print type(msg_class), type(real_topic), type(msg_eval)

	rospy.init_node('listener', anonymous = True)
	rospy.Subscriber(topic, msg_class, callback)
	rospy.spin()



if __name__ == "__main__":
	try_topic()