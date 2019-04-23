#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import String

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
TRIG1 = 27
ECHO1 = 22
GPIO.setup(TRIG1,GPIO.OUT)
GPIO.setup(ECHO1,GPIO.IN)
running = True
rate=rospy.Rate(10)
warningcount1 = 0

def distanceMeasure1():
    rospy.init_node('slave_front_US')
    sFront_US_pub = rospy.Publisher('bt_send_to_master', String, queue_size = 10)
    threshold1 = 2
    GPIO.output(TRIG1,0)
    time.sleep(0.05)
    GPIO.output(TRIG1,1)
    time.sleep(0.00001)
    GPIO.output(TRIG1,0)
    start1 = time.time()
    while GPIO.input(ECHO1) == 0:
        start1 = time.time()
    end1 = time.time()
    while GPIO.input(ECHO1) == 1:
        end1 = time.time()
    duration1 = end1-start1
    distance1 = 171.5*duration1
    distance1 = round(distance1,2)
    if distance1 < threshold1:
        warningcount1 += 1
        if warningcount1 == 5:
            status1 = "sFrontDanger"
            sFront_US_pub.publish(status1)
            warningcount1 = 0
        else:
            status1 = "Warning"
    else:
        warningcount1 = 0
        status1 = "sFrontNormal"
        sFront_US_pub.publish(status1)
#    print "Sensor 1 Distance:",distance1,"m",status1

if __name__=='__main__':
    while running:
        try:
            distanceMeasure1()
            rate.sleep() #see if ros can make it run at 10hz, if not change back to time.sleep(0.01)
        except (KeyboardInterrupt, SystemExit):
            print "Cleaning..."
            GPIO.cleanup()
            print "Bye!"
            sys.exit()

