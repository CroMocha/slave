#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import String

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
TRIG2 = 5
ECHO2 = 6
GPIO.setup(TRIG2,GPIO.OUT)
GPIO.setup(ECHO2,GPIO.IN)
running = True
rate=rospy.Rate(10)
warningcount2 = 0

def distanceMeasure2():
    rospy.init_node('slave_right_US')
    right_US_pub = rospy.Publisher('bt_send_to_master', String, queue_size = 10)
    threshold2 = 2
    GPIO.output(TRIG2,0)
    time.sleep(0.05)
    GPIO.output(TRIG2,1)
    time.sleep(0.00001)
    GPIO.output(TRIG2,0)
    start2 = time.time()
    while GPIO.input(ECHO2) == 0:
        start2 = time.time()
    end2 = time.time()
    while GPIO.input(ECHO2) == 1:
        end2 = time.time()
    duration2 = end2-start2
    distance2 = 171.5*duration2
    distance2 = round(distance2,2)
    if distance2 < threshold2:
        warningcount2 += 1
        if warningcount2 == 5:
            status2 = "RightDanger"
            right_US_pub.publish(status2)
            warningcount2 = 0
        else:
            status2 = "Warning"
    else:
        warningcount2 = 0
        status2 = "RightNormal"
        right_US_pub.publish(status2)
#    print "Sensor 1 Distance:",distance2,"m",status2

if __name__=='__main__':
    while running:
        try:
            distanceMeasure2()
            rate.sleep() #see if ros can make it run at 10hz, if not change back to time.sleep(0.01)
        except (KeyboardInterrupt, SystemExit):
            print "Cleaning..."
            GPIO.cleanup()
            print "Bye!"
            sys.exit()