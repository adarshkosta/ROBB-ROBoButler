# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from sys import argv
from robot import Robot
from serial import Serial

import numpy as np
import time
import cv2
import sys

def nothing(x):
	pass

## Initialization

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()

# Configure our camera
camera.resolution = (320, 240)
camera.framerate = 30

rawCapture = PiRGBArray(camera, size=(320, 240))
 
# allow the camera to warmup
time.sleep(0.1)

# Set default values for the sliders
Hmin, Hmax, Smin, Smax, Vmin, Vmax = 0, 179, 0, 255, 0, 255
lthres, hthres = np.array([Hmin, Smin, Vmin], np.uint8), np.array([Hmax, Smax, Vmax], np.uint8)

## End Initialization

# Set up sliders for image thresholding
def createSliders():
	cv2.namedWindow('Config')
	cv2.createTrackbar('H1', 'Config', 0, 179, nothing)
	cv2.createTrackbar('H2', 'Config', 31, 179, nothing)
	cv2.createTrackbar('S1', 'Config', 0, 255, nothing)
	cv2.createTrackbar('S2', 'Config', 255, 255, nothing)
	cv2.createTrackbar('V1', 'Config', 0, 255, nothing)
	cv2.createTrackbar('V2', 'Config', 255, 255, nothing)

def getSliderValues():
	Hmin = cv2.getTrackbarPos('H1', 'Config')
	Hmax = cv2.getTrackbarPos('H2', 'Config')
	Smin = cv2.getTrackbarPos('S1', 'Config')
	Smax = cv2.getTrackbarPos('S2', 'Config')
	Vmin = cv2.getTrackbarPos('V1', 'Config')
	Vmax = cv2.getTrackbarPos('V2', 'Config')
	
	# Swap around values
	if Hmin > Hmax:
		Hmin, Hmax = Hmax, Hmin

	if Smin > Smax:
		Smin, Smax = Smax, Smin

	if Vmin > Vmax:
		Vmin, Vmax = Vmax, Vmin

	lthres[0], lthres[1], lthres[2] = Hmin, Smin, Vmin
	hthres[0], hthres[1], hthres[2] = Hmax, Smax, Vmax

def do_object_detection(image):
	# Convert it to HSV for thresholding
	hsvimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	# Get slider values
	getSliderValues()

	# Apply threshold to get binary image
	mask = cv2.inRange(hsvimage, lthres, hthres)
	mask1 = cv2.inRange(hsvimage, lthres, hthres)

	# Get contours
	contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# Show test image
	cv2.imshow("Frame3", mask1)

	cx = 160
	area = 0

	for cnt in contours:
		# Get the moments of the contour, and hence calculate the centroid
		# Also calculate a few useful parameters
		area = cv2.contourArea(cnt)
		if (area > 100):
			M = cv2.moments(cnt)
			if (M['m00'] != 0):
				cx = int(M['m10'] / M['m00'])
				cy = int(M['m01'] / M['m00'])

			
			per = cv2.arcLength(cnt, True)

			# Now calculate a rectangle to overlay on top of original image
			x, y, w, h = cv2.boundingRect(cnt)
			cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

	return (image, cx - 160, area);

## Main code begins here

createSliders()

script, rser, user = argv

# Initialize our robot
robot = Serial(rser, 115200, timeout=0)
ser = Serial(user, 921600, timeout=0)

# Set default PWM values
#robot.setFPWM(220)
#robot.setTPWM(180)

buf = ''
state = 0

fc = 0;

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# Grab the image
	image = frame.array
	fc = fc + 1

	# Check if something is available in the serial port
	for c in ser.read():
		buf = buf + c;
		buf = buf.lower()

	if state == 0:
		if buf.find('forward') != -1:
			print buf
			robot.write('forward')
			buf = ''

		if buf.find('back') != -1:
			print buf
			robot.write('back')
			buf = ''

		if buf.find('left') != -1:
			print buf
			robot.write('left')
			buf = ''

		if buf.find('right') != -1:
			print buf
			robot.write('right')
			buf = ''

	if buf.find('box') != -1:
		
		if buf.find('red') != -1:
			# configure thresholds for red here
			#
			pass
		elif buf.find('blue') != -1:
			# configure thresholds for blue here
			#
			pass
		elif buf.find('green') != -1:
			# configure thresholds for green here
			#
			pass
		state = 1
		print buf
		buf = ''

	# Image processing code begins
	# ----------------------------------------------
 	image_processed, dev, area = do_object_detection(image)
	# -----------------------------------------------
	# Image processing code ends

	## If not, take value dev and move left or right
	if state == 1 and fc % 5 == 0:
		if (dev > 20):
			robot.write('right')
			#robot.write('forward')
		elif (dev < -20):
			robot.write('left')
			#robot.moveForward('forward')
		else:
			#robot.moveForward('forward')
			pass

		# Check if Arduino succeeded in gripping object
		gripped = 0
		if gripped == 1:
			state = 0
	
	sys.stdout.write('dev = %03d \r' % dev)
	sys.stdout.flush()

 	# Show the object
	cv2.imshow("Frame2", image_processed)

	# Rest of processing
	key = cv2.waitKey(1) & 0xFF

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

cv2.destroyAllWindows()
