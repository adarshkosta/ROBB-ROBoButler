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

# allow AWB readings to stabilize
#time.sleep(2)

# Freeze exposure and AWB
'''
camera.shutter_speed = 32978L
camera.exposure_mode = 'off'
camera.awb_mode = 'off'
camera.awb_gains = (379 / 256.0, 195 / 128.0)
'''

# Set default values for the sliders
Hmin, Hmax, Smin, Smax, Vmin, Vmax = 0, 179, 0, 255, 0, 255
lthres, hthres = np.array([Hmin, Smin, Vmin], np.uint8), np.array([Hmax, Smax, Vmax], np.uint8)

thres11 = 0.0

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
	cv2.createTrackbar('thres11', 'Config', 1, 255, nothing)
	cv2.createTrackbar('exp', 'Config', 0, 3, nothing)  # Scale of 255

colors = ['red', 'green', 'blue']

def getSliderValues():
	global thres11, color_index
	Hmin = cv2.getTrackbarPos('H1', 'Config')
	Hmax = cv2.getTrackbarPos('H2', 'Config')
	Smin = cv2.getTrackbarPos('S1', 'Config')
	Smax = cv2.getTrackbarPos('S2', 'Config')
	Vmin = cv2.getTrackbarPos('V1', 'Config')
	Vmax = cv2.getTrackbarPos('V2', 'Config')
	thres11 = cv2.getTrackbarPos('thres11', 'Config')
	color_index = cv2.getTrackbarPos('exp', 'Config')

	#camera.exposure_compensation = int(exposure)

	# camera.awb_gains = (awb_gain_num, awb_gain_den)

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
	global hsvimage
	# Convert it to HSV for thresholding
	gfimage = cv2.GaussianBlur(image, (5, 5), 2, 2)
	hsvimage = cv2.cvtColor(gfimage, cv2.COLOR_BGR2HSV)

	# Get slider values
	getSliderValues()

	# Apply threshold to get binary image
	mask = cv2.inRange(hsvimage, lthres, hthres)
	mask1 = mask.copy()

	# Get contours
	contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# Show test image
	cv2.imshow("Frame3", mask1)

	cx = 160
	
	area_max = 0
	area_second_max = 0

	cnt_max = None
	cnt_second_max = None

	i = 0

	for cnt in contours:
		i += 1
		area = cv2.contourArea(cnt)

		if (area > area_max):	
			area_second_max = area_max
			area_max = area

			cnt_second_max = cnt_max
			cnt_max = cnt
		elif (area > area_second_max):
			area_second_max = area
			cnt_second_max = cnt
		else:
			pass

	# Take the max area contour
	if (cnt_max != None):
		M = cv2.moments(cnt_max)
		if (M['m00'] != 0):
			cx = int(M['m10'] / M['m00'])
			cy = int(M['m01'] / M['m00'])

		per = cv2.arcLength(cnt_max, True)
		circularity = (4.0 * 3.1416 * area_max) / (per * per)

		sys.stdout.write('circularity = %f \r' % circularity)
		sys.stdout.flush()

		# Now calculate a rectangle to overlay on top of original image
		x, y, w, h = cv2.boundingRect(cnt_max)
		cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
	else:
		cx = 160
		area = 0

	

	return (image, cx - 160, i, area);

def do_object_detection_2(image):
	global hsvimage
	# Convert it to HSV for thresholding
	gfimage = cv2.GaussianBlur(image, (5, 5), 2, 2)
	hsvimage = cv2.cvtColor(gfimage, cv2.COLOR_BGR2HSV)

	# Get slider values
	getSliderValues()

	# Apply threshold to get binary image
	#mask = cv2.inRange(hsvimage, lthres, hthres)
	
	# Slice through red component of image
	gray = gfimage[:,:,2]
	_,mask = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	mask1 = mask.copy()

	# Show test image
	cv2.imshow("Frame3", mask1)	

	return (gray, 0, 0, 0);

def onmouseevent(event,x,y,flags,param):
	global thresholds
	if event == cv2.EVENT_LBUTTONDBLCLK:
		H = hsvimage[y][x][0]
		S = hsvimage[y][x][1]
		V = hsvimage[y][x][2]
		print('(%d, %d) - H%d, S%d, V%d' % (x, y, H, S, V))

		Hmin = max(H - 10, 0)
		Hmax = min(H + 10, 180)
		Smin = max(S - 50, 0)
		Smax = min(S + 50, 255)
		Vmin = max(V - 60, 0)
		Vmax = min(V + 60, 255)

		cv2.setTrackbarPos('H1', 'Config', Hmin)
		cv2.setTrackbarPos('H2', 'Config', Hmax)
		cv2.setTrackbarPos('S1', 'Config', Smin)
		cv2.setTrackbarPos('S2', 'Config', Smax)
		cv2.setTrackbarPos('V1', 'Config', Vmin)
		cv2.setTrackbarPos('V2', 'Config', Vmax)

		thresholds[colors[color_index]] = np.array([Hmin, Smin, Vmin]), np.array([Hmax, Smax, Vmax])

		i = 1
		for c in colors:
			if thresholds[c] == None:
				i = 0

		if i == 1:
			# Save the thresholds to file
			fil = open('thresholds', 'w')

			for c in colors:
				lth, hth = thresholds[c]
				fil.write(c + ' ' + str(lth[0]) + ' ' + str(lth[1]) + ' ' + str(lth[2]) + ' ' + str(hth[0]) + ' ' + str(hth[1]) + ' ' + str(hth[2]) +  '\n')

			fil.close()





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
robot_buf = ''
state = 0

fc = 0;

gripper_down = 0

# Insert thresholds here
thresholds = {} 

for c in colors:
	thresholds[c] = None

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# Grab the image
	image = frame.array
	fc = fc + 1

	# Check if something is available in the serial port
	for c in ser.read():
		buf = buf + c;
		buf = buf.lower()

	for c in robot.read():
		robot_buf += c
		robot_buf = robot_buf.lower()

	if state == 0:
		if buf.find('forward') != -1:
			print buf
			if buf.find('slight') != -1:
				robot.write('forward 200')
			else:
				robot.write('forward 1000')
			buf = ''

		if buf.find('back') != -1:
			print buf
			if buf.find('slight') != -1:
				robot.write('back 200')
			else:
				robot.write('back 1000')
			buf = ''

		if buf.find('left') != -1:
			print buf
			if buf.find('slight') != -1:
				robot.write('left 200')
			else:
				robot.write('left 700')
			buf = ''

		if buf.find('right') != -1:
			print buf
			if buf.find('slight') != -1:
				robot.write('right 200')
			else:
				robot.write('right 700')
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
 	image_processed, dev, i, area = do_object_detection(image)
	# -----------------------------------------------
	# Image processing code ends

	## If not, take value dev and move left or right
	if state == 1 and fc % 4 == 0:
		if robot_buf.find('gripping') != -1:
			robot_buf = ''
			print('Object gripped successfully')
			gripped = 1
			state = 0

		if (dev > 20):
			robot.write('right 100')
		elif (dev < -20):
			robot.write('left 100')
		else:
			if (gripper_down != 1):
				robot.write('arm down')
				gripper_down = 1

			robot.write('forward 300')

			pass

		# Check if Arduino succeeded in gripping object
		gripped = 0
		if gripped == 1:
			state = 0
	
	sys.stdout.write('dev = %03d \r' % dev)
	sys.stdout.flush()

 	# Show the object
	cv2.imshow('Frame2', image_processed)
	cv2.setMouseCallback('Frame2',onmouseevent)

	# Rest of processing
	key = cv2.waitKey(1) & 0xFF

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	#import code; code.interact(local=locals())

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

cv2.destroyAllWindows()
