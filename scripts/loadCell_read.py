#!/usr/bin/env python

import time
import sys
import RPi.GPIO as GPIO
from hx711 import HX711

GPIO.setwarnings(False)

'''publish results of readings to load cell topic. state machine to subcribe to this topic'''

def readLoadCell():
    rospy.init_node('loadCell')
    loadCell_pub = rospy.Publisher('bt_send_to_master', String, queue_size=10)
    rate = rospy.Rate(10)
    dt = 5 #change data pin
    sck = 6 #change sck pin
    hx = HX711(dt, sck)
    hx.set_reading_format("MSB", "MSB")
    running = True
    max_wt = 2 #set max weight
    net_load = ''
    # HOW TO CALCULATE THE REFFERENCE UNIT
    # To set the reference unit to 1. Put 1kg on your sensor or anything you have and know exactly how much it weights.
    # In this case, 92 is 1 gram because, with 1 as a reference unit I got numbers near 0 without any weight
    # and I got numbers around 184000 when I added 2kg. So, according to the rule of thirds:
    # If 2000 grams is 184000 then 1000 grams is 184000 / 2000 = 92.
    #hx.set_reference_unit(113)
    hx.set_reference_unit(419)

    hx.reset()

    hx.tare()

    print "Tare done! Add weight now..."


    while running:
        try:
            val = hx.get_weight(dt)
            if (val < 0):
                val = 0
            print val

            hx.power_down()
            hx.power_up()
            time.sleep(0.1)

            # Conditions
            if (val >= max_wt):         #If weight over max weight
                print 'net full' #Cue motor command to loosen net
                net_load = 'loadcell.full'
                loadCell_pub.publish(net_load)

            else:      #If weight less than max weight
                print 'net empty' # Cue motor command to tighten net
                net_load = 'loadcell.empty'
                loadCell_pub.publish(net_load)
            rate.sleep()

        except (KeyboardInterrupt, SystemExit):
            print "Cleaning..."
            GPIO.cleanup()
            print "Bye!"
            sys.exit()

if __name__=='__main__':
    readLoadCell()
