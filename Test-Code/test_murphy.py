from flask import Flask, render_template
from flask_ask import Ask, statement, question, audio, current_stream, logger
import time
import serial
import struct
import collections
from copy import copy
import logging
from collections import deque
import numpy as np
import argparse
import socket
import sys
import errno
import threading
import math
from math import atan2, asin
from imutils.video import WebcamVideoStream as VideoStream
import apriltag
import cv2
import imutils
import json
import threading

#ser = serial.Serial('/dev/ttyACM0',9600)
#ser.timeout = 1.0
waiting = True
app = Flask(__name__)
ask = Ask(app, '/')
logging.getLogger('flask_ask').setLevel(logging.INFO)


# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
followFlag = 0
state = 0
goal_x = 0.0
goal_z = 0.0
curr_x = 0.0
curr_z = 0.0
curr_heading = 0.0
heading_offset = 10.0
goal_offset = 0.5
z_pos = 0.0
z_neg = 180.0
x_pos = 90.0
x_neg = -90.0
frame = None
stop = False

HEADER_LENGTH = 10
IP = "192.168.43.59"
PORT = 1234
#my_username = "Murphy"
#print("creating socket")
# Create a socket
# socket.AF_INET - address family, IPv4, some otehr possible are AF_INET6, AF_BLUETOOTH, AF_UNIX
#socket.SOCK_STREAM - TCP, conection-based, socket.SOCK_DGRAM - UDP, connectionless, datagrams, socket.SOCK_RAW - raw IP packets
#client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect to a given ip and port
#client_socket.connect((IP, PORT))
# Set connection to non-blocking state, so .recv() call won;t block, just return some exception we'll handle
#client_socket.setblocking(False)
# Prepare username and header and send them
# We need to encode username to bytes, then count number of bytes and prepare header of fixed size, that we encode to bytes as well

def create_header(strLen, headLen):
    result = "{}".format(strLen)
    resultLen = len(result)
    if resultLen < headLen:
        for x in range(headLen - resultLen) :
            result = result + " "
    return result

frame = None
#vs = VideoStream(src=1).start()
#time.sleep(5.0)
def start_camera():
	global frame
    # keep looping
	print("HELLO CAMERA")
	vs = VideoStream(src=1).start()
	time.sleep(3.0)
	while True:
		time.sleep(.1)
		temp_frame = vs.read()
		if temp_frame is not None:
			temp_frame = cv2.cvtColor(temp_frame, cv2.COLOR_BGR2GRAY)
			frame = temp_frame

#start_camera()


#username = my_username.encode('utf-8')
#username_header = create_header(len(username), HEADER_LENGTH).encode('utf-8')
#client_socket.sendall(username_header + username)
#print("done with socket")


camera_matrix = [[1.78083891e+03, 0.00000000e+00, 9.35628820e+02], [0.00000000e+00, 2.15671011e+03, 5.38832732e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

camera_distortions = [[ 1.41784728e+00, -5.29012388e+01, 4.59024886e-04, 3.03192456e-02,4.97251723e+02]]

camera_distortions = np.array(camera_distortions)
camera_matrix = np.array(camera_matrix)


# Load world points
world_points = {}
with open('worldPoints.json', 'r') as f:
        data = json.load(f)
for k,v in data.items():
        world_points[int(k)] = np.array(v, dtype=np.float32).reshape((4,3,1))

def get_orientation(camera_matrix, R, t):
        proj = camera_matrix.dot(np.hstack((R, t)))
        rot = cv2.decomposeProjectionMatrix(proj)
        rot = rot[-1]
        return rot[1], rot[2], rot[0]


def arduino_write_fail():
        print("Serial write to arduino timed out. Resetting connection")
        ser.close()
        ser.open()
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

def start_app(app):
    app.run(debug=False)

@ask.launch
def launched():
    return question("Hello. what would you like Murphy to do?").reprompt(
        "if you don't need Murphy, please tell him to go to sleep.")

@ask.intent('MoveIntent')
def move(direction):
    msg = ""
    if direction == 'left':
        left()
        time.sleep(1.5)
        halt()
        msg = "Murphy slid to the left"
    elif direction == 'right':
        right()
        time.sleep(1.5)
        halt()
        msg = "Murphy rides to the rightt"
    elif direction == 'forward':
        forward()
        time.sleep(2.0)
        halt()
        msg = "Murphy moved forward boys."
    elif direction == 'backward':
        backward()
        time.sleep(2.0)
        halt()
        msg = "Murphy backs the heck up"
    elif direction == 'halt':
        halt()
        msg = "Murphy has had enough of your hecking crap"
    elif direction == "move":
        return question("In what direction?").reprompt("Can you please give a fooking direction?")
    return question(msg).reprompt("What would you like Murphy to do now?")



@ask.intent('ForwardIntent')
def moveForward():
	forward()
	time.sleep(2.0)
	halt()
	return question("Murphy moved forward boys.").reprompt("What would you like Murphy to do now?")

@ask.intent('LeftIntent')
def moveLeft():
	left()
	time.sleep(1.5)
	halt()
	return question("Murphy slid to the left").reprompt("What would you like Murphy to do now?")

@ask.intent('RightIntent')
def moveRight():
	right()
	time.sleep(1.5)
	halt()
	return question("Murphy rides to the right").reprompt("What would you like Murphy to do now?")

@ask.intent('BackIntent')
def moveBack():
	backward()
	time.sleep(2.0)
	halt()
	return question("Murphy backs the heck up").reprompt("What would you like Murphy to do now?")


@ask.intent('HaltIntent')
def moveHalt():
	global stop
	halt()
	stop = True
	return question("Murphy has had enough of your hecking crap").reprompt("What would you like Murphy to do now?")

@ask.intent('WanderIntent')
def wander_command(command):
    if (command == None):
        wander()
        return question("Murphy is wandering.").reprompt("Encourage Murphy to keep wandering or tell him to stop wandering.")
    if (command == 'keep'):
        wander()
        return question("Woof. Woof").reprompt("Encourage Murphy to keep wandering or tell him to stop wandering.")
    else:
        halt()
        return question("Murphy has stopped wandering.").reprompt("what would you like Murphy to do now?")


@ask.intent('AttackIntent')
def attack():
    tag_seq()
    return question("Perkele!").reprompt("Murphy has calmed down now. What would you like him to do?")

@ask.intent('FollowMeIntent')
def followMeHandler():
	global followFlag
	followFlag = 1
	followthread = 	threading.Thread(target=followPerson, name='followthread')
	followthread.start()
	return question("Murphy is following you now.").reprompt("What Murphy would you Murphy Murphy to Murphy now?")

@ask.intent('StayIntent')
def stayHandler():
	global followFlag
	global stop
	followFlag = 0
	halt()
	stop = True
	return question("Murphy has halted.").reprompt("What would you like Murphy to murph now?")


@ask.intent('RollIntent')
def rollOver():
    right()
    time.sleep(3.0)
    halt()
    return question("Ayyye! Murphy rolled over.").reprompt("What would you like Murphy to do now?")


@ask.intent('AMAZON.FallbackIntent')
def default():
    return question("Sorry, Murphy doesn't understand that command. What would you like him to do?").reprompt(
        "What would you like Murphy to do now?")


@ask.intent('SleepIntent')
def sleep():
    halt()
    print("WE sleepING boiis")
    return statement('Murphy says he is snoring. Bye!')

@ask.intent('SendIntent')
def sendIntent():
    global curr_x
    global curr_z
    message = "distress: " + str(curr_x) + "," + str(curr_z)
    t = threading.Thread(target=send, name='t_send', args=(message,))
    t.start()
    return question('Sent distress signal').reprompt("Are you okay? What can Murphy do to help?").reprompt("The cleaners have been notified. RIP in peace.")


def send(message):
    # Wait for user to input a message
    message = message
    # If message is not empty - send it
    if message:
        # Encode message to bytes, prepare header and convert to bytes, like for username above, then send
        print( "connection lost... reconnecting" )
        connected = False
        # recreate socket
        client_socket = socket.socket()
        while not connected:
            # attempt to reconnect, otherwise sleep for 2 seconds
            try:
                client_socket.connect( (IP, PORT) )
                client_socket.setblocking(False)
                connected = True
                print( "re-connection successful" )
                message = message.encode('utf-8')
                username = "Murphy".encode('utf-8')
                username_header = create_header(len(username), HEADER_LENGTH).encode('utf-8')
                client_socket.sendall(username_header + username)

                message_header = create_header(len(message), HEADER_LENGTH).encode('utf-8')
                client_socket.sendall(message_header + message)    
            except socket.error:
                time.sleep( 2 )
        return

@ask.intent('ReceiveIntent')
def receiveIntent():
    t = threading.Thread(target=receive, name='t_receive')
    t.start()
    t2 = threading.Thread(target=findDistress, name = 't_navigate')
    t2.start()
    return question('Receiving Locations').reprompt("What would you like murphy to murph now?")



def receive():
    global state
    global goal_x
    global goal_z
    global HEADER_LENGTH
    global waiting
    receiving = True
    print("Receiving")
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((IP, PORT))
    time.sleep(0.5)
    username = "Murphy".encode('utf-8')
    header = create_header(len(username), HEADER_LENGTH).encode('utf-8')
    client_socket.sendall(header + username)

    while receiving:
 #       print("REESEEVEENG LOOP")
        try:
           # Now we want to loop over received messages (there might be more than one) and print them
            #username_header = client_socket.recv(HEADER_LENGTH)
  #          print("hello")
            while True:
                time.sleep(2.0)
                username_header = client_socket.recv(HEADER_LENGTH)
                # Receive our "header" containing username length, it's size is defined and constant
                
                # If we received no data, server gracefully closed a connection, for example using socket.close() or socket.shutdown(socket.SHUT_RDWR)
                #if not len(username_header):
                #    print('Connection closed by the server')
                #    sys.exit()
                # Convert header to int value
                username_length = int(username_header.decode('utf-8').strip())
                # Receive and decode username
                username = client_socket.recv(username_length).decode('utf-8')
                # Now do the same for message (as we received username, we received whole message, there's no need to check if it has any length)
                message_header = client_socket.recv(HEADER_LENGTH)
                message_length = int(message_header.decode('utf-8').strip())
                message = client_socket.recv(message_length).decode('utf-8')
                print('\n{} > {}'.format(username, message))
#                print("message[0] = {}".format(message[0]))
                if(message[0] == 'd'):
 #                   print("splitting")
                    m = message.split()
                    m = m[1].split(',')
                    goal_x = float(m[0])
                    goal_z = float(m[1])
                    state = 1
                    receiving = False
                    waiting = False
                    return

        except IOError as e:
            # This is normal on non blocking connections - when there are no incoming data error is going to be raised
            # Some operating systems will indicate that using AGAIN, and some using WOULDBLOCK error code
            # We are going to check for both - if one of them - that's expected, means no incoming data, continue as normal
            # If we got different error code - something happened
            if e.errno != errno.EAGAIN and e.errno != errno.EWOULDBLOCK:
                print('Reading error: {}'.format(str(e)))
                sys.exit()
            # We just did not receive anything
            continue
        except Exception as e:
            print( "connection lost... reconnecting" )
            connected = False
            # recreate socket
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            while not connected:
            # attempt to reconnect, otherwise sleep for 2 seconds
                try:
                    client_socket.connect( (IP, PORT) )
                    #client_socket.setblocking(False)
                    username = "Murphy".encode('utf-8')
                    header = create_header(len(username), HEADER_LENGTH).encode('utf-8')
                    client_socket.sendall(header + username)
                    connected = True
                    print( "re-connection successful" )
                except socket.error:
                    time.sleep( 2 )

def goto(goal_x, goal_z):
    global z_pos
    global x_neg
    global x_pos
    global curr_heading
    global curr_z
    global curr_x
    global stop
    sleep_time = 0.05
    goal_theta = z_pos
    stop = False
    while(abs(curr_heading-goal_theta) > heading_offset and not stop):
        print("finding z")
        right()
        time.sleep(sleep_time)
        get_position()
        time.sleep(sleep_time)
        halt()
    if curr_z > goal_z and not stop:
        while(curr_z-goal_z > goal_offset and not stop):
            print("driving to z")
            print(stop)
            forward()
            time.sleep(sleep_time)
            get_position()
            time.sleep(sleep_time)
            halt()
    else:
        while(goal_z-curr_z > goal_offset and not stop):
            print("driving to z")
            print(stop)
            backward()
            time.sleep(sleep_time)
            get_position()
            time.sleep(sleep_time)
            halt()
    goal_theta = x_pos if goal_x > curr_x else x_neg
    while(abs(curr_heading-goal_theta) > heading_offset and not stop):
        print("finding x")
        right()
        time.sleep(sleep_time)
        get_position()
        time.sleep(sleep_time)
        halt()
    if curr_x > goal_x:
        while(curr_x-goal_x > goal_offset and not stop):
            print("driving to x")
            forward()
            time.sleep(sleep_time)
            get_position()
            time.sleep(sleep_time)
            halt()
    else:
        while(goal_x-curr_x > goal_offset and not stop):
            print("driving to x")
            forward()
            time.sleep(sleep_time)
            get_position() 
            time.sleep(sleep_time)
            halt()
    
    halt()
    
def findDistress():
    global waiting
    waiting = True
    wander()
    print(waiting)
    while(waiting):
        time.sleep(0.5)
    goto(goal_x, goal_z)
    return

def followPerson():
	global followFlag
	time.sleep(2.0)
	detector = apriltag.Detector()

	# keep looping
	while followFlag:
		time.sleep(.1)
		atags = detector.detect(frame)
		temp_origin = np.matrix([[0, 0, 0], [1, 0, 0], [1, -1, 0], [0, -1, 0]])
		found = 0
		for tag in atags:
			corners = tag.corners
			corners = np.array(corners, dtype=np.float32).reshape((4,2,1))
			tag_id = tag.tag_id
			if tag.tag_id == 50:
				found = 1
				break		

		if found:	
			center = tag.center
			x = center[0]
			retval, rvec, tvec = cv2.solvePnP(world_points[tag_id], corners, camera_matrix, camera_distortions)
			rot_matrix, _ = cv2.Rodrigues(rvec)
			R = rot_matrix.transpose()
			pose = -R @ tvec
			if  pose[0] > 10.0:
				if x<150:
					#left()
					print("left")
				elif x>410:
					#right()
					print("right")
				elif x>=150 and x <= 410:
					#forward()
					print("forward")
			else:
				print("You're close enough. halt")
#				halt()
		else:
			print("I can't see you! Turn left")
#			left()
#	halt()

def tag_seq():
    tags = [5, 6, 8, 9]
    while len(tags) > 0:
        print("Going to" + str(tags))
        findTag(tags[0])
        tags.pop(0)
    print("Found")


def findTag(tag):
	time.sleep(2.0)
	detector = apriltag.Detector()

	# keep looping
	while True:
		time.sleep(.1)
		atags = detector.detect(frame)
		temp_origin = np.matrix([[0, 0, 0], [1, 0, 0], [1, -1, 0], [0, -1, 0]])
		found = 0
		for tag in atags:
			corners = tag.corners
			corners = np.array(corners, dtype=np.float32).reshape((4,2,1))
			tag_id = tag.tag_id
			if tag.tag_id == tag:
				found = 1
				break		

		if found:	
			center = tag.center
			x = center[0]
			retval, rvec, tvec = cv2.solvePnP(world_points[tag_id], corners, camera_matrix, camera_distortions)
			rot_matrix, _ = cv2.Rodrigues(rvec)
			R = rot_matrix.transpose()
			pose = -R @ tvec
			if  pose[0] > 10.0:
				if x<150:
					#left()
					print("left")
				elif x>410:
					#right()
					print("right")
				elif x>=150 and x <= 410:
					#forward()
					print("forward")
			else:
				print("You're close enough. halt")
                return True
#				halt()
		else:
			print("I can't see you! Turn left")
#			left()
#	halt()


def get_position():
    global frame
    global curr_x
    global curr_z
    global curr_heading
    detector = apriltag.Detector()
    atags = detector.detect(frame)	
#	print(atag)
    yaw_bar = 0.0
    x_bar = 0.0
    z_bar = 0.0
    atags = [a for a in atags if a.tag_id != 50]
    for tag in atags:
        corners = tag.corners
        corners = np.array(corners, dtype=np.float32).reshape((4,2,1))
        tag_id = tag.tag_id
        retval, rvec, tvec = cv2.solvePnP(world_points[tag_id], corners, camera_matrix, camera_distortions)
        rot_matrix, _ = cv2.Rodrigues(rvec)
        R = rot_matrix.transpose()
        pose = -R @ tvec
        x_bar += pose[0]
        z_bar += pose[2]
        yaw, pitch, roll = get_orientation(camera_matrix, R, tvec)
        yaw_bar += yaw
		#print("Yaw: {} \n Pitch: {} \n Roll: {}".format(yaw,pitch,roll))
    if len(atags) > 1:
        curr_heading = yaw_bar/len(atags)
        curr_x = x_bar/len(atags)
        curr_z = z_bar/len(atags)
    else:
        curr_heading = yaw_bar
        curr_x = x_bar
        curr_z = z_bar
    print(curr_heading)
    print(curr_x)
    print(curr_z)

camthread = threading.Thread(target=start_camera, name='camthread')
camthread.start()
while frame is None:
	time.sleep(0.5)	
print("starting app")

# tag_seq()
murphythread = threading.Thread(target=start_app, name = 'murphythread', args=(app,))
murphythread.setDaemon(True)
murphythread.start()



#t = threading.Thread(target=receive, name='t_receive')
#t.start()
#t2 = threading.Thread(target=findDistress, name = 't_navigate')
#t2.start()


