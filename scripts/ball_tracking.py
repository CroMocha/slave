from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
from math import tan, atan

redLower = (155, 75, 50)
redUpper = (255, 255, 255)
greenLower = (50, 50, 50)
greenUpper = (100, 150, 255)

ball_width = 0.083
pole_width = 0.021
pole_height = 0.5
focal_length = 213 #For wide-angle lens
fov = np.pi/2 #Our calculated FOV for wide angle lens is 90 degrees
image_width = 320
image_height = 240

vs = VideoStream(src=0).start()
# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
while True:
	# grab the current frame
	frame = vs.read()

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=320)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	ball_mask = cv2.inRange(hsv, redLower, redUpper)
	ball_mask = cv2.erode(ball_mask, None, iterations=2)
	ball_mask = cv2.dilate(ball_mask, None, iterations=2)

    	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	ball_cnts = cv2.findContours(ball_mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	ball_cnts = imutils.grab_contours(ball_cnts)
	center = None

	# only proceed if at least one contour was found
	if len(ball_cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		ball_c = max(ball_cnts, key=cv2.contourArea)
		((ball_x, ball_y), radius) = cv2.minEnclosingCircle(ball_c)
		M = cv2.moments(ball_c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 5:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
            		diameter = radius*2
			dist_from_img_centre = ball_x - image_width/2
            		ball_distance = (ball_width*focal_length/diameter)
            		ball_tan_angle = 2*dist_from_img_centre*tan(0.5*fov)/image_width
            		ball_angle = atan(ball_tan_angle)
			print "-----------------------------------------"
            		print "x: " + str(dist_from_img_centre) + "y: " + str(ball_y) + "radius:" + str(radius)
            		print "distance: " + str(ball_distance) + "   angle: " + str(ball_angle)
			cv2.circle(frame, (int(ball_x), int(ball_y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# otherwise, release the camera
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()
