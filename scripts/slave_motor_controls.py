#!/usr/bin/env python
import RPi.GPIO as gpio
import time
import rospy
from aquascrub.msg import apriltag

ena = 2 #enable motor 1
enb = 3 #enable motor 2
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
lmotor = gpio.PWM(ena,50)
rmotor = gpio.PWM(enb,50)
lmotor.start(0)
rmotor.start(0)
gpio.output(in1, True)
gpio.output(in2, False)
gpio.output(in3, False)
gpio.output(in4, True)



class SlaveMotorControl:
    def __init__(self):
        # PID control for forward/backward motion
        self.kp = 100
        self.ki = 0
        self.kd = 0
        # PID control for yaw motion
        self.yawrate_kp = 0
        self.yawrate_ki = 0
        self.yawrate_kd = 0
        # For the d term in PID controller
        self.prev_x = 0
        self.prev_y = 0
        self.prev_pitch = 0
        # For the I term in PID controller
        self.accumulated_x = 0
        self.accumulated_y = 0
        self.accumulated_pitch = 0
        self.integralCap_x = 10000
        self.integralCap_y = 10000
        self.integralCap_pitch = 10000
        # If slave receives command from master
        self.reference_spd = 0
        self.max_spd = 100
        self.min_spd = -100
        rospy.init_node('slave_motor_cmd', anonymous=True) #name the node
        #rospy.Subscriber('bt_receive_from_master', velocity, self.speed_from_master, queue_size = 1) #callback to some function, increase queuesize to handle more messages
        rospy.Subscriber('apriltag', apriltag, self.maintain_orientation, queue_size = 1)
        rospy.spin()

    # def speed_from_master(self,data):
    #     vel_msg = data.spd
    #     spd_msg = vel_msg.split()
    #     lmotor_spd = spd_msg[0]
    #     rmotor_spd = spd_msg[1]
    #     print "lmotor_spd is" + spd_msg[0]
    #     print "rmotor_spd is" + spd_msg[1]
    #     lmotor.ChangeDutyCycle(int(lmotor_spd))
    #     rmotor.ChangeDutyCycle(int(rmotor_spd))

    def maintain_orientation(self,data):
        x = data.x
        y = data.y
        pitch = data.pitch
        diff_x = data.x - self.prev_x
        diff_y = data.y - self.prev_y
        diff_pitch = data.pitch - self.prev_pitch
        self.accumulated_x += x
        self.accumulated_y += y
        self.accumulated_pitch += pitch

        corrected_spd = (self.kp*y) + (self.kd*diff_y) + (self.kd*self.accumulated_y)
        corrected_yawrate = (self.yawrate_kp*pitch) + (self.yawrate_kd*diff_pitch) + (self.yawrate_ki*self.accumulated_pitch)
        output_spd = self.reference_spd + corrected_spd

        # Adding constraints to maximum/minimum speed
        lmotor_spd = output_spd + corrected_yawrate
        if (lmotor_spd > self.max_spd):
            lmotor_spd = self.max_spd
        if (lmotor_spd < self.min_spd):
            lmotor_spd = self.min_spd
        rmotor_spd = output_spd - corrected_yawrate
        if (rmotor_spd > self.max_spd):
            rmotor_spd = self.max_spd
        if (rmotor_spd < self.min_spd):
            rmotor_spd = self.min_spd

        # Changing direction of motors
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
        print(lmotor_spd)
        print(rmotor_spd)
        lmotor.ChangeDutyCycle(int(abs(lmotor_spd)))
        rmotor.ChangeDutyCycle(int(abs(rmotor_spd)))


        self.prev_x = x
        self.prev_y = y
        self.prev_pitch = pitch

        if(self.accumulated_x >= self.integralCap_x):
            self.accumulated_x = integralCap_x
        if(self.accumulated_y >= self.integralCap_y):
            self.accumulated_y = integralCap_y
        if(self.accumulated_pitch >= self.integralCap_pitch):
            self.accumulated_pitch = integralCap_pitch


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
