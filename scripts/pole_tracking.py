from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
from math import tan, atan

redLower = (155, 75, 50)
redUpper = (255, 255, 255)
greenLower = (50, 55, 16)
greenUpper = (77, 255, 255)

ball_width = 0.083
pole_width = 0.11
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
	original_frame = vs.read()

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(original_frame, width=320)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	pole_mask = cv2.inRange(hsv, greenLower, greenUpper)
	pole_mask = cv2.erode(pole_mask, None, iterations=2)
	pole_mask = cv2.dilate(pole_mask, None, iterations=2)

    	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	pole_cnts = cv2.findContours(pole_mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	pole_cnts = imutils.grab_contours(pole_cnts)

	# only proceed if at least one contour was found
	if len(pole_cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(pole_cnts, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
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
		dist_from_img_centre = pole_x - image_width/2
		pole_tan_angle = 2*dist_from_img_centre*tan(0.5*fov)/image_width
                pole_angle = atan(pole_tan_angle)
		print "x: " + str(pole_x) + "   y: " + str(pole_y) + "   width: " + str(pole_pixel_width) + "   height: " + str(pole_pixel_height)
		print "pole_distance: " + str(pole_distance) + "   pole_angle: " + str(pole_angle)

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
