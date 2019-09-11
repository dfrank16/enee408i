from flask import Flask, render_template
from flask_ask import Ask, statement, question
import thread


# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import serial
import struct
import csv
# open serial 
valueToWrite = 250
ser = serial.Serial('/dev/ttyACM0',9600)
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=32,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
yellowLower = (29, 86, 6)
yellowUpper = (64, 255, 255)
pts = deque(maxlen=args["buffer"])

# Download the helper library from https://www.twilio.com/docs/python/install
from twilio.rest import Client


# Your Account Sid and Auth Token from twilio.com/console
account_sid = 'AC1654dafcc8c2b4a21f119f18bf5951ad'
auth_token = '16d3b1e44a9094c66bb41d535075e6bd'
client = Client(account_sid, auth_token)

global globalCommand
global heartRate
global alert
global globalMessage
global globalMessageReprompt
global doneAlertRespond
heartRate = 60
doneAlertRespond = 0
alert = 0
globalMessage = ""
globalMessageReprompt = ""

# end packages for follow me

app = Flask(__name__)
ask = Ask(app, '/')

def halt():
	valueToWrite= 0
	ser.write(struct.pack('>B', valueToWrite))

def right():
	valueToWrite= 2
	ser.write(struct.pack('>B', valueToWrite))

def left():
	valueToWrite= 1
	ser.write(struct.pack('>B', valueToWrite))

def forward():
	valueToWrite= 3
	ser.write(struct.pack('>B', valueToWrite))

def backward():
	valueToWrite= 4
	ser.write(struct.pack('>B', valueToWrite))
	
def wander():
	valueToWrite= 5
	ser.write(struct.pack('>B', valueToWrite))

def followLoop():
	global alert
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	vs = cv2.VideoCapture(1)
	# allow the camera to warm up
	time.sleep(2.0)
	global globalCommand
	while (True):
		if (globalCommand == "stop") or (globalCommand == "alert"):
			vs.release()
			open = vs.isOpened()
			halt()
			break
		# grab the current frame
		_, frame = vs.read()	
		

		# handle the frame from VideoCapture or VideoStream
		frame = frame[1] if args.get("video", False) else frame
			
		# if we are viewing a video and we did not grab a frame,
		# then we have reached the end of the video
		if frame is None:
			break
		# resize the frame, blur it, and convert it to the HSV
		# color space
		frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)	

		# construct a mask for the color "green", then perform	
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, yellowLower, yellowUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]
		center = None
		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# only proceed if the radius meets a minimum size
#			if radius > 10:
#				# draw the circle and centroid on the frame,
#				# then update the list of tracked points
#				cv2.circle(frame, (int(x), int(y)), int(radius),
#					(0, 255, 255), 2)
#				cv2.circle(frame, center, 5, (0, 0, 255), -1)
		
		if (center is None):
			halt()
			continue
		# Move left or right
		if (center[0] < 100):
			# turn very left]
			left()
	
		elif (center[0] >=100) and (center[0] < 200):
			# turn a little left
			left()
		elif (center[0] >=200) and (center[0] < 400):
			#  Move forward/backward/stop
			if (radius < 75) :
				# move forward
				forward()

			elif (radius > 150):
				# move backwards
				backward()
			else :
				halt()
		elif (center[0] >=400) and (center[0] < 500):
			# turn a little right
			right()

		elif (center[0] >= 500):
			# turn very right
			right()

		# update the points queue
		pts.appendleft(center)

def findLoop():
	global alert
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	vs = cv2.VideoCapture(1)
	# allow the camera to warm up
	time.sleep(2.0)
	numLoops = 0
	while (True):
		# grab the current frame
		_, frame = vs.read()

		# handle the frame from VideoCapture or VideoStream
		frame = frame[1] if args.get("video", False) else frame
	
		if frame is None:
			vs.release
			break
		# resize the frame, blur it, and convert it to the HSV
		# color space
		frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)	

		# construct a mask for the color "green", then perform	
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, yellowLower, yellowUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]
		center = None
		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# only proceed if the radius meets a minimum size
#			if radius > 10:
#				# draw the circle and centroid on the frame,
#				# then update the list of tracked points
#				cv2.circle(frame, (int(x), int(y)), int(radius),
#					(0, 255, 255), 2)
#				cv2.circle(frame, center, 5, (0, 0, 255), -1)

		# radius here defines how big the shirt must be on the screen to count as "found"
		if ((center is None) or (radius < 30)):
			if (numLoops > 150):
				halt()
				vs.release
				return "Sparky couldn't find you. try getting closer to Sparky or getting in his field of view."
			# turn left
			left()
#			continue
		# Move left or right
		else:
			if (center[0] < 200):
				# turn left
				left()
			elif (center[0] > 400):
				# turn right
				right()
			else:
				halt()
				vs.release
				return "Sparky found you!"

		# update the points queue
		pts.appendleft(center)
	
		numLoops = numLoops + 1

def comeHereLoop():
	global alert
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	radius = 0
	vs = cv2.VideoCapture(1)
	sizeRadHere = 100
	# allow the camera to warm up
	time.sleep(2.0)
	while (radius < sizeRadHere):
		# grab the current frame
		_, frame = vs.read()	

		# handle the frame from VideoCapture or VideoStream
		frame = frame[1] if args.get("video", False) else frame
	
		# if we are viewing a video and we did not grab a frame,
		# then we have reached the end of the video
		if frame is None:
			break
		# resize the frame, blur it, and convert it to the HSV
		# color space
		frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)	

		# construct a mask for the color "green", then perform	
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, yellowLower, yellowUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]
		center = None
		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		if (center is None):
			halt()
			continue
		# Move left or right
		if (center[0] < 200) :
			# turn left
			left()
		elif (center[0] >=200) and (center[0] < 400) :
			#  Move forward/backward/stop
			if (radius < sizeRadHere) :
				# move forward
				forward()

			else :
				halt()
				vs.release
				return "Sparky is here!"
		elif (center[0] >=400):
			# turn right
			right()

	# update the points queue
	pts.appendleft(center)
	vs.release
	return "Sparky is here!"

def updateHR():
	global heartRate
	global alert
	alert = 0
	global globalCommand
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	
	with open('heartRate.csv') as heartRateFile:
		#time.sleep(1.0)
		heartRateData = list(csv.reader(heartRateFile))
		heartRate = 0.0
		lastAlert = 0
		i = 0
		prev_i = 0
		while True:
			time.sleep(1.0)
			lastAlert = alert
			heartRateStr = heartRateData[i][0]
			heartRate = float(heartRateStr)
			print heartRate
			if ((heartRate < 50) or (heartRate > 110)) and ((i - prev_i > 30) or (prev_i == 0)):
				prev_i = i
				alert = 1
				doneAlertRespond = 0
				if (lastAlert == 0):
					globalCommand = "alert"
					halt()
					time.sleep(1.0)
					responseAlert = respondAlert()
					if (responseAlert == "Sparky found you"):
						globalMessage = ("Sparky found you. Your heartrate is abnormal. Are you okay?")
						globalMessageReprompt = ("Say no i'm not to trigger an alert, or yes i am to dismiss")
					else:
						# send the alert
						text()
						globalMessage = responseAlert
						globalMessageReprompt = "What would you like Sparky to do now?"
						
					doneAlertRespond = 1
			i += 1

def monitorHR():
	thread.start_new_thread(updateHR, ())
	return

def respondAlert():
	found = findLoop()
	if (found == "Sparky found you!"):
		return ("Sparky found you")
	elif (found == "alert"):
		return question(globalMessage).reprompt(globalMessageReprompt)
	else:
		return ("Sparky could not find you. He is sending an alert.")

def text():
	account_sid = 'AC1654dafcc8c2b4a21f119f18bf5951ad'
	auth_token = '16d3b1e44a9094c66bb41d535075e6bd'
	client = Client(account_sid, auth_token)

	message = client.messages \
    	.create(
        	body="Katrina's heartrate has dropped dangerously low, and she says she needs help.",
        	from_='+14105054636',
        	to='+13013672497'
    	)

@ask.launch
def launched():
	#monitorHR()
	return question("hello. what would you like Sparky to do?").reprompt("if you don't need Sparky, please tell him to go to sleep.")

@ask.intent('AMAZON.FallbackIntent')
def default():
	return question("Sorry, Sparky doesn't understand that command. What would you like him to do?").reprompt("What would you like Sparky to do now?")

@ask.intent('MoveIntent')
def move(direction):
	global alert
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	if (alert):
		halt()
		globalCommand = "alert"
		while (doneAlertRespond == 0):
			pass
		alert = 0
		return question(globalMessage).reprompt(globalMessageReprompt)
	else:
		msg = ""
		if direction == 'left':
			left()
			time.sleep(1.0)
			halt()
			msg = "Sparky moved left"
		elif direction == 'right':
			right()
			time.sleep(1.0)
			halt()
			msg = "Sparky moved right"
		elif direction == 'forward':
			forward()
			time.sleep(4.0)
			halt()
			msg = "Sparky moved forward"
		elif direction == 'backward':
			backward()
			time.sleep(2.0)
			halt()
			msg = "Sparky moved backward"
		elif direction == 'halt':
			halt()
			msg = "Sparky has stopped moving"
		elif direction == "move":
			return question("In what direction?").reprompt("Can you please give a direction?")	
		return question(msg).reprompt("what would you like Sparky to do now?")

@ask.intent('FollowIntent')
def followMe(command):
	global globalCommand
	globalCommand = command
	global alert
	global globalMessage
	global doneAlertRespond

	if (alert):
		halt()
		globalCommand = "alert"
		while (doneAlertRespond == 0):
			pass
		alert = 0
		return question(globalMessage).reprompt(globalMessageReprompt)

	if (globalCommand != "stop"):
		thread.start_new_thread(followLoop, ())
		time.sleep(8.0)
	else:
		halt()
	# if a video path was not supplied, grab the reference
	# to the webcam
	
	if (globalCommand == "keep"):
		return question("Woof.").reprompt("Tell Sparky to stop following you or encourage him to keep following you.")
	elif (globalCommand == "stop"):
		return question("Sparky stopped following you").reprompt("What would you like Sparky to do now?")
	elif (globalCommand == "alert"):
		if (alert):
			halt()
			while (doneAlertRespond == 0):
				pass
			alert = 0
			return question(globalMessage).reprompt(globalMessageReprompt)
	else:
		return question('Sparky says woof. He is following you.').reprompt("Tell Sparky to stop following you or encourage him to keep following you.")

@ask.intent('FindIntent')
def findMe():
	global alert
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	if (alert):
		halt()
		globalCommand = "alert"
		while (doneAlertRespond == 0):
			pass
		alert = 0
		return question(globalMessage).reprompt(globalMessageReprompt)
	else:
		msg = findLoop()
		halt()
		if (msg == "alert"):
			return question(globalMessage).reprompt(globalMessageReprompt)
		return question(msg).reprompt("can Sparky help you with anything else?")
	
@ask.intent('ComeIntent')
def comeHere():
	global alert
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	if (alert):
		halt()
		globalCommand = "alert"
		while (doneAlertRespond == 0):
			pass
		alert = 0
		return question(globalMessage).reprompt(globalMessageReprompt)
	else:
		msg = findLoop()
		if (msg == "Sparky found you!"):
			thread.start_new_thread(comeHereLoop, ())
		else:
			return question(msg).reprompt("what would you like Sparky to do now?")
		halt()
		#return question(msg)
		return question("Sparky is on his way.").reprompt("what would you like Sparky to do now?")

@ask.intent('WanderIntent')
def wander_command(command):
	global alert
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	globalCommand = "stop"
	if (alert):
		halt()
		globalCommand = "alert"
		while (doneAlertRespond == 0):
			pass
		alert = 0
		return question(globalMessage).reprompt(globalMessageReprompt)

	else:
		if (command == None):
			wander()
			return question("Sparky is wandering.").reprompt("Encourage Sparky to keep wandering or tell him to stop wandering.")
		if (command == 'keep'):
			wander()
			return question("woof.").reprompt("Encourage Sparky to keep wandering or tell him to stop wandering.")
		else:
			halt()
			return question("Sparky has stopped wandering.").reprompt("what would you like Sparky to do now?")

@ask.intent('AttackIntent')
def attack():
	global alert
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	if (alert):
		halt()
		globalCommand = "alert"
		while (doneAlertRespond == 0):
			pass
		alert = 0
		return question(globalMessage).reprompt(globalMessageReprompt)
	else:
		return question("woof woof woof woof gur.").reprompt("Sparky has calmed down now. What would you like him to do?")
	
@ask.intent('RollIntent')
def rollOver():
	global alert
	global globalMessage
	global globalMessageReprompt
	global doneAlertRespond
	if (alert):
		halt()
		globalCommand = "alert"
		while (doneAlertRespond == 0):
			pass
		alert = 0
		return question(globalMessage).reprompt(globalMessageReprompt)
	else:
		right()
		time.sleep(6.0)
		halt()
		return question("woof! Sparky rolled over.").reprompt("What would you like Sparky to do now?")

@ask.intent('OkayIntent')
def okay(response):
	if (response == "yes"):
		return question("Sparky is glad you're okay.").reprompt("What would you like Sparky to do next?")
	elif (response == "no"):
		# send the alert
		text()
		return question("Sparky is sending an alert.").reprompt("What would you like Sparky to do next? With no response, Sparky will go to sleep.")
	else:
		return question("Sorry, I couldn't understand that. Are you okay?").reprompt("Sparky is worried about you. Please respond with yes i am or no i'm not")

@ask.intent('MonitorIntent')
def monitorHRintent():
	global globalMessage
	global globalMessageReprompt
	thread.start_new_thread(updateHR, ())
	time.sleep(1.0)
	return question("Sparky is monitoring your heart rate").reprompt("What would you like Sparky to do now?")
	#return question(globalMessage).reprompt(globalMessageReprompt)

@ask.intent('SleepIntent')
def sleep():
	halt()
	return statement('Sparky says woof woof. Bye!')

if __name__ == '__main__':
	app.run(debug=True)