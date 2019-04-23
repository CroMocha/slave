#!/usr/bin/env python
import RPi.GPIO as gpio
import time
import rospy
from geometry_msgs.msg import Twist


ena = 2 #enable motor 1 (rightmotor)
enb = 3 #enable motor 2 (leftmotor)
in1 = 17 #motor 1 +
in2 = 22 #motor 1 -
in3 = 23 #motor 2 +
in4 = 24 #motor 2 -

gpio.setwarnings(False)
gpio.setmode(gpio.BCM)
gpio.setup(in1, gpio.OUT)
gpio.setup(in2, gpio.OUT)
gpio.setup(in3, gpio.OUT)
gpio.setup(in4, gpio.OUT)
gpio.setup(ena, gpio.OUT)
gpio.setup(enb, gpio.OUT)
rmotor = gpio.PWM(ena,50)
lmotor = gpio.PWM(enb,50)
lmotor.start(0)
rmotor.start(0)
gpio.output(in1, True)
gpio.output(in2, False)
gpio.output(in3, False)
gpio.output(in4, True)

linear_multiplier = 150
angular_multiplier = 25



class SlaveMotorControl:
    def __init__(self):
        rospy.init_node('control', anonymous=True) #name the node
        #rospy.Subscriber('bt_receive_from_master', velocity, self.speed_from_master, queue_size = 1) #callback to some function, increase queuesize to handle more messages
        rospy.Subscriber('cmd_vel', Twist, self.move, queue_size = 1)
        rospy.spin()

    def move(self,data):
        linear = data.linear.x
        angular = data.angular.z

        lmotor_spd = linear*linear_multiplier - angular*angular_multiplier
        rmotor_spd = linear*linear_multiplier + angular*angular_multiplier


        if (lmotor_spd > self.max_spd):
            lmotor_spd = self.max_spd
        if (lmotor_spd < self.min_spd):
            lmotor_spd = self.min_spd
        if (0 < lmotor_spd < 40):
            lmotor_spd = 40
        if (-40 < lmotor_spd < 0):
            lmotor_spd = -40


        if (rmotor_spd > self.max_spd):
            rmotor_spd = self.max_spd
        if (rmotor_spd < self.min_spd):
            rmotor_spd = self.min_spd
        if (0 < rmotor_spd < 40):
            rmotor_spd = 40
        if (-40 < rmotor_spd < 0):
            rmotor_spd = -40

        if (lmotor_spd < 0):
            gpio.output(in1, False)
            gpio.output(in2, True)
        elif (lmotor_spd >= 0):
            gpio.output(in1, True)
            gpio.output(in2, False)
        if (rmotor_spd < 0):
            gpio.output(in3, True)
            gpio.output(in4, False)
        elif (rmotor_spd >= 0):
            gpio.output(in3, False)
            gpio.output(in4, True)

        # Changing speed of motors
        print "-----------------------------------------------------------------"
        print "leftmotor: " + str(lmotor_spd) + "  rightmotor: " + str(rmotor_spd)
        lmotor.ChangeDutyCycle(int(abs(lmotor_spd)))
        rmotor.ChangeDutyCycle(int(abs(rmotor_spd)))


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
