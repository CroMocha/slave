#!/usr/bin/env python
import rospy
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import sys
from math import tan, atan
from aquascrub.msg import object

redLower = (155, 75, 50)
redUpper = (255, 255, 255)
greenLower = (50, 55, 16)
greenUpper = (77, 255, 255)
yellowLower = (0,0,0)
yellowUpper = (255,255,255)

ball_width = 0.083
pole_width = 0.11
pole_height = 0.5
collection_height = 0.000
focal_length = 213 #For wide-angle lens
fov = np.pi/2 #Our calculated FOV for wide angle lens is 90 degrees
image_width = 320
image_height = 240

vs = VideoStream(src=0).start()
# allow the camera or video file to warm up
time.sleep(2.0)

def trackObjects():
    rospy.init_node('object_tracking')
    object_pub = rospy.Publisher('objects', object, queue_size=10)

    # keep looping
    while True:
        try:
        	# grab the current frame
        	frame = vs.read()
                object_msg = object()
        	# resize the frame, blur it, and convert it to the HSV
        	# color space
        	frame = imutils.resize(frame, width=320)
        	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        	ball_mask = cv2.inRange(hsv, redLower, redUpper)
        	ball_mask = cv2.erode(ball_mask, None, iterations=2)
        	ball_mask = cv2.dilate(ball_mask, None, iterations=2)

        	ball_cnts = cv2.findContours(ball_mask.copy(), cv2.RETR_EXTERNAL,
        		cv2.CHAIN_APPROX_SIMPLE)
        	ball_cnts = imutils.grab_contours(ball_cnts)
        	center = None

                pole_mask = cv2.inRange(hsv, greenLower, greenUpper)
        	pole_mask = cv2.erode(pole_mask, None, iterations=2)
        	pole_mask = cv2.dilate(pole_mask, None, iterations=2)

                pole_cnts = cv2.findContours(pole_mask.copy(), cv2.RETR_EXTERNAL,
        		cv2.CHAIN_APPROX_SIMPLE)
        	pole_cnts = imutils.grab_contours(pole_cnts)

            collection_mask = cv2.inRange(hsv, yellowLower, yellowUpper)
        collection_mask = cv2.erode(collection_mask, None, iterations=2)
        collection_mask = cv2.dilate(collection_mask, None, iterations=2)

            collection_cnts = cv2.findContours(collection_mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        collection_cnts = imutils.grab_contours(collection_cnts)

                if len(ball_cnts) > 0 or len(pole_cnts) > 0 or len(collection_cnts) > 0:
                    print "-------------------------------------------------"

        	if len(ball_cnts) > 0:

        		ball_c = max(ball_cnts, key=cv2.contourArea)
        		((ball_x, ball_y), radius) = cv2.minEnclosingCircle(ball_c)
        		M = cv2.moments(ball_c)
        		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        		if radius > 5:
                            diameter = radius*2
              	            ball_dist_from_img_centre = ball_x - image_width/2
                            ball_distance = (ball_width*focal_length/diameter)
                            ball_tan_angle = 2*ball_dist_from_img_centre*tan(0.5*fov)/image_width
                            ball_angle = atan(ball_tan_angle)
                            print "ball_x: " + str(ball_dist_from_img_centre) + "ball_y: " + str(ball_y) + "radius:" + str(radius)
                            print "ball_distance: " + str(ball_distance) + "   ball_angle: " + str(ball_angle)
        		    cv2.circle(frame, (int(ball_x), int(ball_y)), int(radius), (0, 255, 255), 2)
        		    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                            object_msg.trashdetected = True
                            object_msg.trashangle = ball_angle
                            object_msg.trashdist = ball_distance

                else:
                    object_msg.trashdetected = False
                    object_msg.trashangle = 0.0
                    object_msg.trashdist = 0.0


                if len(pole_cnts) > 0:
        		# find the largest contour in the mask, then use
        		# it to compute the minimum enclosing circle and
        		# centroid
        		pole_c = max(pole_cnts, key=cv2.contourArea)
                        rect = cv2.minAreaRect(pole_c)
        		pole_x = rect[0][0]
        		pole_y = rect[0][1]
        		pole_pixel_width = min(rect[1][0],rect[1][1])
        		pole_pixel_height = max(rect[1][0],rect[1][1])
        		angle_rotation = rect[2]
        		box = cv2.boxPoints(rect)
        		box = np.int0(box)
        		cv2.drawContours(frame,[box],0,(0,0,255),2)
        		cv2.circle(frame, (int(pole_x),int(pole_y)), 3, (0, 255, 0), -1)
        		pole_distance = (pole_height*focal_length/pole_pixel_height)
        		pole_dist_from_img_centre = pole_x - image_width/2
        		pole_tan_angle = 2*pole_dist_from_img_centre*tan(0.5*fov)/image_width
                        pole_angle = atan(pole_tan_angle)
        		print "x: " + str(pole_x) + "   y: " + str(pole_y) + "   width: " + str(pole_pixel_width) + "   height: " + str(pole_pixel_height)
        		print "pole_distance: " + str(pole_distance) + "   pole_angle: " + str(pole_angle)
                        object_msg.obsdetected = True
                        object_msg.obsangle = pole_angle
                        object_msg.obsdist = pole_distance

                else:
                    object_msg.obsdetected = False
                    object_msg.obsangle = 0.0
                    object_msg.obsdist = 0.0

                if len(collection_cnts) > 0:
        		# find the largest contour in the mask, then use
        		# it to compute the minimum enclosing circle and
        		# centroid
        		collection_c = max(collection_cnts, key=cv2.contourArea)
                        collection_rect = cv2.minAreaRect(collection_c)
        		collection_x = collection_rect[0][0]
        		collection_y = collection_rect[0][1]
        		collection_width = min(collection_rect[1][0],collection_rect[1][1])
        		collection_pixel_height = max(collection_rect[1][0],collection_rect[1][1])
        		collection_box = cv2.boxPoints(collection_rect)
        		collection_box = np.int0(collection_box)
        		cv2.drawContours(frame,[collection_box],0,(255,0,255),2)
        		cv2.circle(frame, (int(collection_x),int(collection_y)), 3, (0, 255, 0), -1)
        		collection_distance = (collection_height*focal_length/collection_pixel_height)
        		collection_dist_from_img_centre = collection_x - image_width/2
        		collection_tan_angle = 2*collection_dist_from_img_centre*tan(0.5*fov)/image_width
                        collection_angle = atan(collection_tan_angle)
        		print "x: " + str(collection_x) + "   y: " + str(collection_y) + "   width: " + str(collection_width) + "   height: " + str(collection_pixel_height)
        		print "collection_distance: " + str(collection_distance) + "   collection_angle: " + str(collection_angle)
                        object_msg.collectiondetected = True
                        object_msg.collectionangle = collection_angle
                        object_msg.collectiondist = collection_distance

                else:
                    object_msg.collectiondetected = False
                    object_msg.collectionangle = 0.0
                    object_msg.collectiondist = 0.0

                object_pub.publish(object_msg)

                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
        	    break

        except (KeyboardInterrupt, SystemExit):
    	    vs.release()
            cv2.destroyAllWindows()
            sys.exit()

if __name__=='__main__':
    trackObjects()
