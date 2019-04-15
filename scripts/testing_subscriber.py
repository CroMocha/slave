#!/usr/bin/env python
import RPi.GPIO as gpio
import time
import rospy
from aquascrub.msg import apriltag


class SlaveMotorControl:
    def __init__(self):
        rospy.init_node('slave_motor_cmd', anonymous=True) #name the node
        #rospy.Subscriber('bt_receive_from_master', velocity, self.speed_from_master, queue_size = 1) #callback to some function, increase queuesize to handle more messages
        rospy.Subscriber('apriltag', apriltag, self.speed_from_master, queue_size = 1)
        rospy.spin()

    def speed_from_master(self,data):
        vel_msg = data.x
        print "x is" + vel_msg

if __name__ == '__main__':
    try:
        f = SlaveMotorControl()
    except KeyboardInterrupt:
        print "cleanup gpio"
        gpio.cleanup()
    except EOFError:
        print "quiting"
        gpio.cleanup()
        exit()
