
# A line following robot for Robotics Systems final project
# Coded using Python 3 and OpenCV 3.3 for a Raspberry Pi 3B running Raspbian
# Uses Raspberry Pi camera module v2

from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
import time
import cv2
import picamera
import numpy as np

# Initialize camera
camera = picamera.PiCamera()
camera.resolution = (192,108)
camera.framerate = 20
rawCapture = PiRGBArray(camera,size=(192,108))
time.sleep(0.1)

# setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

# Loop over all frames captured by camera indefinitely
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

	# Display camera input
	image = frame.array
	cv2.imshow('img',image)

	# Create key to break for loop
	key = cv2.waitKey(1) & 0xFF

	# convert to grayscale for easier processing.
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	#smoothen the image using Gaussian blur via(i/p img, (dimensions of kernal, must be odd and positive), standard_deviation)
	blur = cv2.GaussianBlur(gray,(5,5),0)
	# converts the image according to the threshold values, fourth arg reverses the colors since we need white obj in black bg for finding contours.
	ret,thresh1 = cv2.threshold(blur,100,255,cv2.THRESH_BINARY_INV)

	# Erode to eliminate noise, by removing a bit of the edges of the image, like feathering.(img,kernal.iter)
	mask = cv2.erode(thresh1, None, iterations=2)
	#Dilate to restore eroded parts of image as eroding shrinks the image.(img,kernal.iter)
	mask = cv2.dilate(mask, None, iterations=2)
	
	# this can be done in one step using opening.
	# opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
	
	# Find all contours in frame,(img.copy,relation_find,compress_stuff)
	something, contours, hierarchy = cv2.findContours(mask.copy(),1,cv2.CHAIN_APPROX_NONE)

	# Find x-axis centroid of largest contour and cut power to appropriate motor
	# to recenter camera on centroid.
	if len(contours) > 0:
		# Find largest contour area and image moments
		c = max(contours, key = cv2.contourArea)
		M = cv2.moments(c)

		# Find x-axis centroid using image moments and convert them to integer. Centroid is given by the relations, Cx=[M10/M00] and Cy=[M01/M00]
		cx = int(M['m10']/M['m00'])
		# the width value is from 0 to 190 so lets say the road/path is in the middle 40-150.
		if cx >= 150:
			GPIO.output(12, GPIO.LOW)
			GPIO.output(21, GPIO.HIGH)

		if cx < 150 and cx > 40:
			GPIO.output(12, GPIO.HIGH)
			GPIO.output(21, GPIO.HIGH)

		if cx <= 40:
			GPIO.output(12, GPIO.HIGH)
			GPIO.output(21, GPIO.LOW)

	if key == ord("q"):
            break

	rawCapture.truncate(0)

#Turn off motors
GPIO.output(12, GPIO.LOW)
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(21, GPIO.LOW)

#remove assigned values of pins
GPIO.cleanup()
