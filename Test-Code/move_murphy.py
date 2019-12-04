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
import math
import multiprocessing as mp
cvQueue1, cvQueue2 = mp.JoinableQueue(), mp.JoinableQueue()

ser = serial.Serial('/dev/ttyACM0',9600)
ser.timeout = 0.05
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
drive_state = 0
frame = None
world_points = {}
#camera_matrix = [[1.78083891e+03, 0.00000000e+00, 9.35628820e+02], [0.00000000e+00, 2.15671011e+03, 5.38832732e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
camera_matrix = [[1102.2685765043407, 0.0, 1005.8822521774567], [0.0, 1104.9939493307077, 592.2828735697244], [0.0, 0.0, 1.0]]
#camera_distortions = [[ 1.41784728e+00, -5.29012388e+01, 4.59024886e-04, 3.03192456e-02,4.97251723e+02]]
camera_distortions = [[-0.08959985635048899], [0.0001713488859657656], [-0.06283449521732573], [0.04151141258421837]]
camera_distortions = np.array(camera_distortions)
camera_matrix = np.array(camera_matrix)
tag_sequence = []

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

#vs = VideoStream(src=1).start()
#time.sleep(5.0)
def start_camera(queue1, queue2):
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
			if not queue1.empty():
				print("florb")
				florb = queue1.get()
				queue1.task_done()
				queue2.put(frame)
				queue2.join()

#start_camera()


#username = my_username.encode('utf-8')
#username_header = create_header(len(username), HEADER_LENGTH).encode('utf-8')
#client_socket.sendall(username_header + username)
#print("done with socket")


#Given a point (z,x), find the id of the closest apriltag
def findClosestTag(z,x):
    min_dist = 10000
    min_tag = -1
    for key in world_points.keys():
        tag = world_points[key]
        z_diff = abs(z - tag[0][0])
        x_diff = abs(x - tag[0][2])

        dist = math.sqrt((z_diff**2) + (x_diff**2))
        if dist < min_dist:
            min_dist = dist
            min_tag = key
    return min_tag


# Load world points
with open('worldPoints.json', 'r') as f:
        data = json.load(f)

for k,v in data.items():
        world_points[int(k)] = np.array(v, dtype=np.float32).reshape((4,3,1))


with open('tagSequence', 'r') as f:
    for line in f.readlines():
        tag_sequence.append(int(line))


def get_orientation(camera_matrix, R, t):
        proj = camera_matrix.dot(np.hstack((R, t)))
        rot = cv2.decomposeProjectionMatrix(proj)
        rot = rot[-1]
        return rot[1], rot[2], rot[0]

def get_pose(corners, points):
        global camera_matrix
        global camera_distortions
        corners = np.array(corners, dtype=np.float32).reshape((4,2,1))
        points = np.array(points, dtype=np.float32).reshape((4,3,1))
        retval, rvec, tvec = cv2.solvePnP(points, corners, camera_matrix, camera_distortions)
        rot_matrix, _ = cv2.Rodrigues(rvec)
        R = rot_matrix.transpose()
        pose = -R @ tvec
        return pose

def arduino_write_fail():
        print("Serial write to arduino timed out. Resetting connection")
        ser.close()
        ser.open()

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
    global followFlag
    global stop
    halt()
    stop = True
    followFlag = 0
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
    goto_thread = threading.Thread(target=goto, args=(15,0,), name="goto")
    goto_thread.start()
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
    global stop
    stop = True
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

#given a current tag_id and a target tag_id, return the id of the next tag in the sequence from current to target
def getNextTag(current,target):
    global tag_sequence
    curr_i = tag_sequence.index(current)
    target_i = tag_sequence.index(target)
    if curr_i < target_i:
        return tag_sequence[curr_i + 1]
    else:
        return tag_sequence[curr_i - 1]

#def findTag(target):
#    if target == -1:

# Drive to the tag with the given tag_id. If -1 is passed in, we will drive to the first tag we see.
# returns the id of the tag we've driven to.
def goto_tag(target):
    global ser
    global stop
    stop_time = 0.1
    step_time = 0.75
    step_counter = 0
    #world origin is used for each tag to determine relative distance from Murphy to the tag
    temp_origin = np.matrix([[0, 0, 0], [1, 0, 0], [1, -1, 0], [0, -1, 0]])
    last = "forward"

    print("Attempting to navigate to tag #{}".format(target))
    #If -1 is passed as the target, we will lock onto the first tag we see. Could be improved
    target_tag = None if target == -1 else target
    #Navigation loop: can be interrupted by setting global variable stop, set in 'halt' and 'stay' intents
    while not stop:   
        ser.reset_output_buffer()         
        time.sleep(0.1)
        cvQueue1.put(1)
        cvQueue1.join()
        time.sleep(0.1)
        frame = cvQueue2.get()
        cvQueue2.task_done()
        detector = apriltag.Detector()
        print("Looking for tag #{}".format(target_tag))
        found = 0
        atags = detector.detect(frame)
        for tag in atags: #look for our target
            if target_tag is not None:
                if tag.tag_id == target_tag:
                    found = 1
                    print("Found")
                    break
            else:#if target is undefined, we will lock onto the first tag we see
                target_tag = tag.tag_id
                found = 1
                print("Found")
                break
        if found:
            #try to keep the center of the tag in the center of the frame
            print("I see tag #{}".format(target_tag))
            x = tag.center[0]
            pose = get_pose(tag.corners, temp_origin)
            print("Pose[0]: {}".format(pose[0]))
            if  abs(pose[0]) > 10.0:
                if x<150:
                    left()
                    print("left")
                    time.sleep(stop_time)
                    last = "left"
                elif x>410:
                    right()
                    print("right")
                    time.sleep(stop_time)
                    last = "right"
                elif x>=150 and x <= 410:
                    forward()
                    print("forward")
                    time.sleep(0.1)
            else:
                print("You're close enough. halt and return")
                halt()
                if last == "left":
                    right()
                    time.sleep(0.3)
                    backward()
                    time.sleep(0.2)
                    halt()
                elif last == "right":
                    left()
                    time.sleep(0.3)
                    backward()
                    time.sleep(0.2)
                    halt()
                return target_tag
        else:
            #Search for the target tag if we can't see it.
            #TODO: Add more complex/better search code for when we can't see the target
            print("I can't see you! Turn left")
            
            if step_counter < 10:
                left()
           # time.sleep(0.075)
                time.sleep(step_time)
                halt()
                step_counter += 1
            else:
                ser.close()
                ser.open()
                wander()
                time.sleep(7)
                step_counter = 0
    halt()


def goto(goal_x, goal_z):
    global stop
    sleep_time = 0.05
    #stop = False
    print("Received command to go to x = {}, y = {}".format(goal_x, goal_z))
    #Determine our final target
    target_tag = findClosestTag(goal_z,goal_x)
    #Get to some starting tag
    print("Determined the target tag is tag #{}".format(target_tag))
    current_tag = -1

    cvQueue1.put(1)
    cvQueue1.join()
    time.sleep(0.1)
    frame = cvQueue2.get()
    cvQueue2.task_done()
    detector = apriltag.Detector()
    atags = detector.detect(frame)
    print(atags)
    delta = 100
    for tag in atags:
        if abs(tag_sequence.index(tag.tag_id) - tag_sequence.index(target_tag)) < delta:
            print(tag.tag_id)
            delta = abs(tag_sequence.index(tag.tag_id) - tag_sequence.index(target_tag))
            current_tag = tag.tag_id



    while (not stop) and (current_tag is not target_tag):
        #figure out what the next tag we need to drive to is
        next_tag = getNextTag(current_tag, target_tag)
        #drive to next target
        print("Next tag: {}, Current Tag:{}".format(next_tag,current_tag))
        current_tag = goto_tag(next_tag)
        #if current tag is our final target, we're done.
        if current_tag == target_tag:
            print("Target acquired: We're here")
            break
    halt()



def findDistress():
    global waiting
    global stop
    waiting = True
    wander()
    print(waiting)
    while(waiting):
        time.sleep(0.5)
    stop = False
    goto(goal_x, goal_z)
    return

def followPerson():
    global stop
    global followFlag
    time.sleep(1.0)
    cvQueue1.put(1)
    cvQueue1.join()
    time.sleep(1.0)
    frame = cvQueue2.get()
    cvQueue2.task_done()
    detector = apriltag.Detector()

	# keep looping
    while followFlag and (not stop):
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
            retval, rvec, tvec = cv2.solvePnP(temp_origin, corners, camera_matrix, camera_distortions)
            rot_matrix, _ = cv2.Rodrigues(rvec)
            R = rot_matrix.transpose()
            pose = -R @ tvec
            if  pose[0] > 10.0:
                if x<150:
                    left()
                    print("left")
                elif x>410:
                    right()
                    print("right")
                elif x>=150 and x <= 410:
                    forward()
                    print("forward")
            else:
                print("You're close enough. halt")
                halt()
        else:
            print("I can't see you! Turn left")
            left()
    halt()


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

if __name__ == "__main__":
    camthread = mp.Process(target=start_camera, name='camthread', args=(cvQueue1, cvQueue2,))
    camthread.start()
    time.sleep(0.5)
    print("starting app")
    murphythread = threading.Thread(target=start_app, name = 'murphythread', args=(app,))
    murphythread.setDaemon(True)
    murphythread.start()



#t = threading.Thread(target=receive, name='t_receive')
#t.start()
#t2 = threading.Thread(target=findDistress, name = 't_navigate')
#t2.start()
