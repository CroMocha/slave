#!/usr/bin/env python
import RPi.GPIO as gpio
import time
import rospy
from aquascrub.msg import apriltag

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



class SlaveMotorControl:
    def __init__(self):
        # PID control for forward/backward motion
        self.kp = 150
        self.ki = 0
        self.kd = 0
        # PID control for forward/backward motion
        self.y_kp = 0
        self.y_ki = 0
        self.y_kd = 0
        # For the d term in PID controller
        self.prev_dist_error = 0
        self.prev_y = 0
        # For the I term in PID controller
        self.accumulated_dist_error = 0
        self.accumulated_y = 0

        self.integralCap_dist_error = 10000
        self.integralCap_y = 10000

        # If slave receives command from master
        self.reference_spd = 75
        self.max_spd = 100
        self.min_spd = -100
        self.ref_distance = 1.0
        rospy.init_node('slave_motor_cmd', anonymous=True) #name the node
        #rospy.Subscriber('bt_receive_from_master', velocity, self.speed_from_master, queue_size = 1) #callback to some function, increase queuesize to handle more messages
        rospy.Subscriber('apriltag', apriltag, self.maintain_orientation, queue_size = 1)
        rospy.spin()


    def maintain_orientation(self,data):
        distance = data.distance
        dist_error = self.ref_distance - distance
        diff_dist_error = disterror - self.prev_dist_error
        self.accumulated_dist_error += dist_error

        y = data.y
        diff_y = data.y - self.prev_y
        self.accumulated_y += y

        corrected_spd = (self.kp*dist_error) + (self.kd*diff_dist_error) + (self.ki*self.accumulated_dist_error)
        catchup_spd = (self.y_kp*y) + (self.y_kd*diff_y) + (self.y_ki*self.accumulated_y)

        # Adding constraints to maximum/minimum speed
        lmotor_spd = self.reference_spd + corrected_spd + catchup_spd
        if (lmotor_spd > self.max_spd):
            lmotor_spd = self.max_spd
        if (lmotor_spd < self.min_spd):
            lmotor_spd = self.min_spd
        # if (0 < lmotor_spd < 40):
        #     lmotor_spd = 40
        # if (-40 < lmotor_spd < 0):
        #     lmotor_spd = -40
        lmotor_spd *= 0.8

        rmotor_spd = output_spd - corrected_spd + catchup_spd
        if (rmotor_spd > self.max_spd):
            rmotor_spd = self.max_spd
        if (rmotor_spd < self.min_spd):
            rmotor_spd = self.min_spd
        # if (0 < rmotor_spd < 40):
        #     rmotor_spd = 40
        # if (-40 < rmotor_spd < 0):
        #     rmotor_spd = -40

        #rmotor_spd *= 0.72
        # Changing direction of motors
        if (rmotor_spd < 0):
            gpio.output(in1, False)
            gpio.output(in2, True)
        elif (rmotor_spd >= 0):
            gpio.output(in1, True)
            gpio.output(in2, False)
        if (lmotor_spd < 0):
            gpio.output(in3, True)
            gpio.output(in4, False)
        elif (lmotor_spd >= 0):
            gpio.output(in3, False)
            gpio.output(in4, True)

        # Changing speed of motors
        print "-----------------------------------------------------------------"
        print "pitch: " + str(pitch) + "   x: " + str(x) + "   y: " + str(y)
        print "leftmotor: " + str(lmotor_spd) + "  rightmotor: " + str(rmotor_spd)
        lmotor.ChangeDutyCycle(int(abs(lmotor_spd)))
        rmotor.ChangeDutyCycle(int(abs(rmotor_spd)))


        self.prev_dist_error = dist_error
        self.prev_y = y

        if(self.accumulated_dist_error >= self.integralCap_dist_error):
            self.accumulated_dist_error = integralCap_dist_error
        if(self.accumulated_y >= self.integralCap_y):
            self.accumulated_y = integralCap_y

        if(abs(dist_error) < 0.2):
            self.accumulated_dist_error = 0
        if(abs(y)<0.1):
            self.accumulated_y = 0


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
