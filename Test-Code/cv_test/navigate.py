 
# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from math import atan2, asin
from collections import deque
from imutils.video import VideoStream
import apriltag
import numpy
import argparse
import cv2
import imutils
import time
import json
import struct
from flask import Flask, render_template
#from flask_ask import Ask
import serial
arduino = serial.Serial('/dev/ttyACM0', 9600)
arduino.timeout=0.3
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())


def arduino_write_fail():
	print("Serial write to arduino timed out. Resetting connection")
	arduino.close()
	arduino.open()
drive_state = 0
def halt():
	global drive_state
	if drive_state != 0:
		try:
			ser.write(struct.pack('>B', 0))
			drive_state = 0
		except:
			arduino_write_fail()

def left():
	global drive_state
	if drive_state != 1:
		try:
			ser.write(struct.pack('>B', 1))
			drive_state = 1
		except:
			arduino_write_fail()
def right():
	global drive_state
	if drive_state != 2:
		try:
			ser.write(struct.pack('>B', 2))
			drive_state = 2
		except:
			arduino_write_fail()

def forward():
	global drive_state
	if drive_state != 3:
		try:
			ser.write(struct.pack('>B', 3))
			drive_state = 3
		except:
			arduino_write_fail()
def backward():
	global drive_state
	if drive_state != 4:
		try:
			ser.write(struct.pack('>B', 4))
			drive_state = 4
		except:
			arduino_write_fail()
def wander():
	global drive_state
	if drive_state != 5:
		try:
			ser.write(struct.pack('>B', 5))
			drive_state = 5
		except:
			arduino_write_fail()

print("Hello precam")
if not args.get("video", False):
	vs = VideoStream(src=1).start()
	print("WebCam Capture Successful")
	#vs = cv2.VideoCapture(1)
# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])
print("Hello postcam")
# allow the camera or video file to warm up
time.sleep(2.0)


detector = apriltag.Detector()

state = 0
camera_matrix = [[1.78083891e+03, 0.00000000e+00, 9.35628820e+02], [0.00000000e+00, 2.15671011e+03, 5.38832732e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

camera_distortions = [[ 1.41784728e+00, -5.29012388e+01, 4.59024886e-04, 3.03192456e-02,4.97251723e+02]]

camera_distortions = numpy.array(camera_distortions)
camera_matrix = numpy.array(camera_matrix)

follow = 1 

# Load world points
world_points = {}
with open('worldPoints.json', 'r') as f:
	data = json.load(f)
for k,v in data.items():
	world_points[int(k)] = numpy.array(v, dtype=numpy.float32).reshape((4,3,1))

def get_orientation(camera_matrix, R, t):
	proj = camera_matrix.dot(numpy.hstack((R, t)))
	rot = cv2.decomposeProjectionMatrix(proj)
	rot = rot[-1]
	return rot[1], rot[2], rot[0]

# keep looping
while True:

	time.sleep(.1)
	# grab the current frame
	frame = vs.read()
	#frame = vs.grab()
	# handle the frame from VideoCapture or VideoStream
	#frame = frame[1]# if args.get("video", False) else frame

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break

	# resize the frame, blur it, and convert it to the HSV
	# color space
	cv2.imshow("frame",frame)
	frame = imutils.resize(frame, width=600)
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	#cv2.imshow("frame", frame)
	atags = detector.detect(frame)	
#	print(atag)
	temp_origin = numpy.matrix([[0, 0, 0], [1, 0, 0], [1, -1, 0], [0, -1, 0]])
	for tag in atags:	
		corners = tag.corners
		corners = numpy.array(corners, dtype=numpy.float32).reshape((4,2,1))
		tag_id = tag.tag_id
		retval, rvec, tvec = cv2.solvePnP(world_points[tag_id], corners, camera_matrix, camera_distortions)
		rot_matrix, _ = cv2.Rodrigues(rvec)
		R = rot_matrix.transpose()
		pose = -R @ tvec
		yaw, pitch, roll = get_orientation(camera_matrix, R, tvec)
		#print("Yaw: {} \n Pitch: {} \n Roll: {}".format(yaw,pitch,roll))
		print("Pose: {}\n\n".format(pose))
		if follow and tag.tag_id == 50:
			center = tag.center
			x = center[0]
			print(x)
			if  pose[2] > 5.0:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                #       cv2.circle(frame, (int(x), int(y)), int(radius),
                        #       (0, 255, 255), 2)
                #       cv2.circle(frame, center, 5, (0, 0, 255), -1)

				if x<150:
					left()
				elif x>410:
					right()
				elif x>=150 and x <= 410:
					forward()
			else:
				halt()

	# Q to quit
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break


# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()
	#vs.release()
# otherwise, release the camera
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()
