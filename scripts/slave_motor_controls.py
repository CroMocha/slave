#!/usr/bin/env python
import RPi.GPIO as gpio
import time
import rospy
from aquascrub.msg import velocity

ena = 2 #enable motor 1
enb = 3 #enable motor 2
in1 = 17 #motor 1 +
in2 = 22 #motor 1 -
in3 = 23 #motor 2 +
in4 = 24 #motor 2 -

gpio.setmode(gpio.BCM)
gpio.setup(in1, gpio.OUT)
gpio.setup(in2, gpio.OUT)
gpio.setup(in3, gpio.OUT)
gpio.setup(in4, gpio.OUT)
gpio.setup(ena, gpio.OUT)
gpio.setup(enb, gpio.OUT)
lmotor = gpio.PWM(ena,50)
rmotor = gpio.PWM(enb,50)
lmotor.start(0)
rmotor.start(0)
gpio.output(in1, True)
gpio.output(in2, False)
gpio.output(in3, True) 
gpio.output(in4, False)

class SlaveMotorControl:
    def __init__(self):
        rospy.init_node('slave_motor_cmd', anonymous=True) #name the node
        rospy.Subscriber('bt_receive_from_master', velocity, self.speed_from_master, queue_size = 1) #callback to some function, increase queuesize to handle more messages
        rospy.spin()

    def speed_from_master(self,data):
        vel_msg = data.spd
        spd_msg = vel_msg.split()
        lmotor_spd = spd_msg[0]
        rmotor_spd = spd_msg[1]
        print "lmotor_spd is" + spd_msg[0]
        print "rmotor_spd is" + spd_msg[1]
        lmotor.ChangeDutyCycle(int(lmotor_spd))
        rmotor.ChangeDutyCycle(int(rmotor_spd))
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