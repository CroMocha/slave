from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import apriltag
import cv2

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)
 
# allow the camera to warmup
time.sleep(1)
 
# grab an image from the camera
camera.capture(rawCapture1, format="bgr")
camera.close()
image = rawCapture.array
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
detector = apriltag.Detector()
result = detector.detect(gray)