#!/usr/bin/env python

import rospy
from dbw_mkz_msgs.msg import SteeringReport, BrakeReport, ThrottleReport, TurnSignal
import tty, termios, sys

def getchar():
   #Returns a single character from standard input
   fd = sys.stdin.fileno()
   old_settings = termios.tcgetattr(fd)
   try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
   finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
   return ch
   

def speed_display_tester():
    pubSteering = rospy.Publisher('test_steering', SteeringReport, queue_size=10)
    pubBrake = rospy.Publisher('test_brake', BrakeReport, queue_size=10)
    pubTrottle = rospy.Publisher('test_trottle', ThrottleReport, queue_size=10)
    pubSignal = rospy.Publisher('test_signal', TurnSignal, queue_size=10)
    rospy.init_node('vehicle_control_tester', anonymous = False)
    rate = rospy.Rate(10)
    
    steering = 0.0
    brake = 0.15
    throttle = 0.15
    signal = 0
    while not rospy.is_shutdown():
        ch = getchar()
        if (ch == 'q'): signal = 1
        elif (ch == 'e'): signal = 2
        elif (ch == 'a'): steering -= 0.1
        elif (ch == 'd'): steering += 0.1
        elif (ch == 'w'): throttle += 0.03
        elif (ch == 's'): brake += 0.03
        elif (ch == 'p'): break
        else:
            throttle -= 0.03
            brake -= 0.03

        if (throttle > 0.5): throttle = 0.5
        elif (throttle < 0.15): throttle = 0.15

        if (brake > 0.5): brake = 0.5
        elif (brake < 0.15): brake = 0.15

        msgSteering = SteeringReport()
        msgBrake = BrakeReport()
        msgThrottle = ThrottleReport()
        msgSignal = TurnSignal()
        msgSteering.steering_wheel_angle = steering
        msgBrake.pedal_input = brake
        msgThrottle.pedal_input = throttle
        msgSignal.value = signal
        pubSteering.publish(msgSteering)
        pubBrake.publish(msgBrake)
        pubTrottle.publish(msgThrottle)
        pubSignal.publish(msgSignal)
        rate.sleep()



if __name__ == "__main__":
    try:
        speed_display_tester()
    except rospy.ROSInterruptException:
        pass
