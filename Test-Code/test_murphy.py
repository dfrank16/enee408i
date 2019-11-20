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


# ser = serial.Serial('/dev/ttyACM0',9600)

waiting = True
app = Flask(__name__)
ask = Ask(app, '/')
logging.getLogger('flask_ask').setLevel(logging.INFO)

 
# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages

state = 0
goal_x = 0.0
goal_z = 0.0
curr_x = 0.0
curr_z = 0.0
curr_heading = 0.0
heading_offset = math.pi/6.0
goal_offset = 0.5
theta_x_add = 0.0
theta_x_dec = math.pi
theta_z_add = math.pi/2.0
theta_z_dec = (3.0)*math.pi/2.0

HEADER_LENGTH = 10
IP = "192.168.43.193"
PORT = 1234
my_username = "Murphy"
# Create a socket
# socket.AF_INET - address family, IPv4, some otehr possible are AF_INET6, AF_BLUETOOTH, AF_UNIX
# socket.SOCK_STREAM - TCP, conection-based, socket.SOCK_DGRAM - UDP, connectionless, datagrams, socket.SOCK_RAW - raw IP packets
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect to a given ip and port
client_socket.connect((IP, PORT))
# Set connection to non-blocking state, so .recv() call won;t block, just return some exception we'll handle
client_socket.setblocking(False)
# Prepare username and header and send them
# We need to encode username to bytes, then count number of bytes and prepare header of fixed size, that we encode to bytes as well
username = my_username.encode('utf-8')
username_header = f"{len(username):<{HEADER_LENGTH}}".encode('utf-8')
client_socket.send(username_header + username)








playlist = [
    # 'https://www.freesound.org/data/previews/367/367142_2188-lq.mp3',
    'https://archive.org/download/mailboxbadgerdrumsamplesvolume2/Ringing.mp3',
    'https://archive.org/download/petescott20160927/20160927%20RC300-53-127.0bpm.mp3',
    'https://archive.org/download/plpl011/plpl011_05-johnny_ripper-rain.mp3',
    'https://archive.org/download/piano_by_maxmsp/beats107.mp3',
    'https://archive.org/download/petescott20160927/20160927%20RC300-58-115.1bpm.mp3',
    'https://archive.org/download/PianoScale/PianoScale.mp3',
    # 'https://archive.org/download/FemaleVoiceSample/Female_VoiceTalent_demo.mp4',
    'https://archive.org/download/mailboxbadgerdrumsamplesvolume2/Risset%20Drum%201.mp3',
    'https://archive.org/download/mailboxbadgerdrumsamplesvolume2/Submarine.mp3',
    # 'https://ia800203.us.archive.org/27/items/CarelessWhisper_435/CarelessWhisper.ogg'
]


def halt():
	valueToWrite= 0
	ser.write(struct.pack('>B', valueToWrite))

def left():
	valueToWrite= 1
	ser.write(struct.pack('>B', valueToWrite))

def right():
	valueToWrite= 2
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

def start_app():
    app.run(debug=True)

class QueueManager(object):
    """Manages queue data in a seperate context from current_stream.
    The flask-ask Local current_stream refers only to the current data from Alexa requests and Skill Responses.
    Alexa Skills Kit does not provide enqueued or stream-histroy data and does not provide a session attribute
    when delivering AudioPlayer Requests.
    This class is used to maintain accurate control of multiple streams,
    so that the user may send Intents to move throughout a queue.
    """

    def __init__(self, urls):
        self._urls = urls
        self._queued = collections.deque(urls)
        self._history = collections.deque()
        self._current = None

    @property
    def status(self):
        status = {
            'Current Position': self.current_position,
            'Current URL': self.current,
            'Next URL': self.up_next,
            'Previous': self.previous,
            'History': list(self.history)
        }
        return status

    @property
    def up_next(self):
        """Returns the url at the front of the queue"""
        qcopy = copy(self._queued)
        try:
            return qcopy.popleft()
        except IndexError:
            return None

    @property
    def current(self):
        return self._current

    @current.setter
    def current(self, url):
        self._save_to_history()
        self._current = url

    @property
    def history(self):
        return self._history

    @property
    def previous(self):
        history = copy(self.history)
        try:
            return history.pop()
        except IndexError:
            return None

    def add(self, url):
        self._urls.append(url)
        self._queued.append(url)

    def extend(self, urls):
        self._urls.extend(urls)
        self._queued.extend(urls)

    def _save_to_history(self):
        if self._current:
            self._history.append(self._current)

    def end_current(self):
        self._save_to_history()
        self._current = None

    def step(self):
        self.end_current()
        self._current = self._queued.popleft()
        return self._current

    def step_back(self):
        self._queued.appendleft(self._current)
        self._current = self._history.pop()
        return self._current

    def reset(self):
        self._queued = collections.deque(self._urls)
        self._history = []

    def start(self):
        self.__init__(self._urls)
        return self.step()

    @property
    def current_position(self):
        return len(self._history) + 1

queue = QueueManager(playlist)

@ask.launch
def launched():
    return question("Hello. what would you like Murphy to do?").reprompt(
        "if you don't need Murphy, please tell him to go to sleep.")

@ask.intent('PlayMusicIntent')
def playMusic():
    speech = 'Heres a playlist of some sounds. You can ask me Next, Previous, or Start Over'
    stream_url = queue.start()
    return audio(speech).play(stream_url)


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
	halt()
	return question("Murphy has had enough of your crap").reprompt("What would you like Murphy to do now?")

@ask.intent('WanderIntent')
def wander_command(command):
    if (command == None):
        wander()
        return question("Murphy is wandering.").reprompt("Encourage Murphy to keep wandering or tell him to stop wandering.")
    if (command == 'keep'):
        wander()
        return question("woof.").reprompt("Encourage Murphy to keep wandering or tell him to stop wandering.")
    else:
        halt()
        return question("Murphy has stopped wandering.").reprompt("what would you like Murphy to do now?")


@ask.intent('AttackIntent')
def attack():
    return question("Drop your weapon. You have 10 seconds to comply. Die. Die. Die. Die. Die. Die. Die. Die. Die. Die. Die.").reprompt("Murphy has calmed down now. What would you like him to do?")


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
    return statement('Sent distress signal')


def send(message):
    # Wait for user to input a message
    message = message
    # If message is not empty - send it
    if message:
        # Encode message to bytes, prepare header and convert to bytes, like for username above, then send
        message = message.encode('utf-8')
        message_header = f"{len(message):<{HEADER_LENGTH}}".encode('utf-8')
        client_socket.send(message_header + message)
        return statement('Location Sent')

@ask.intent('ReceiveIntent')
def receiveIntent():
    t = threading.Thread(target=receive, name='t_receive')
    t.start()
    return statement('Receiving Locations')



def receive():
    global state
    global goal_x
    global goal_z
    receiving = True
    while receiving:
        try:
           # Now we want to loop over received messages (there might be more than one) and print them
            while True:
                # Receive our "header" containing username length, it's size is defined and constant
                username_header = client_socket.recv(HEADER_LENGTH)
                # If we received no data, server gracefully closed a connection, for example using socket.close() or socket.shutdown(socket.SHUT_RDWR)
                if not len(username_header):
                    print('Connection closed by the server')
                    sys.exit()
                # Convert header to int value
                username_length = int(username_header.decode('utf-8').strip())
                # Receive and decode username
                username = client_socket.recv(username_length).decode('utf-8')
                # Now do the same for message (as we received username, we received whole message, there's no need to check if it has any length)
                message_header = client_socket.recv(HEADER_LENGTH)
                message_length = int(message_header.decode('utf-8').strip())
                message = client_socket.recv(message_length).decode('utf-8')
                # Print message
                print(f'\n{username} > {message}')
                print(my_username + ' > ')
                if(message[0] == 'd'):
                    print("splitting")
                    m = message.split()
                    m = m[1].split(',')
                    goal_x = float(m[0])
                    goal_z = float(m[1])
                    state = 1
                    receiving = False
                    return 

        except IOError as e:
            # This is normal on non blocking connections - when there are no incoming data error is going to be raised
            # Some operating systems will indicate that using AGAIN, and some using WOULDBLOCK error code
            # We are going to check for both - if one of them - that's expected, means no incoming data, continue as normal
            # If we got different error code - something happened
            if e.errno != errno.EAGAIN and e.errno != errno.EWOULDBLOCK:
                print('Reading error: {}'.format(str(e)))
                return statement('Nothing Received')
            # We just did not receive anything
            continue
        except Exception as e:
            # Any other exception - something happened, exit
            print('Reading error: {}'.format(str(e)))
            return statement('Unexpected Error')


def findDistress():
    wander()
    while(not waiting):
        time.sleep(0.5)
    if goal_x > curr_x:
        while(abs(curr_heading-theta_x_add) > heading_offset):
            right()
            time.sleep(0.15)
            halt()
    else:
        while(abs(curr_heading-theta_x_dec) > heading_offset):
            right()
            time.sleep(0.15)
            halt()
    while(abs(curr_x-goal_x) > goal_offset):
        forward()
        time.sleep(0.15)
        halt()
    if goal_z > curr_z:
        while(abs(curr_heading-theta_z_add) > heading_offset):
            right()
            time.sleep(0.15)
            halt()
    else:
        while(abs(curr_heading-theta_z_dec) > heading_offset):
            right()
            time.sleep(0.15)
            halt()
    while(abs(curr_z-goal_z) > goal_offset):
        forward()
        time.sleep(0.15)
        halt()
    halt()
    return


while(waiting):
    if state == 0:
        # Wander
        # arduino.write(struct.pack('>B', 9))
        t2 = threading.Thread(target=receive, name='t2')
        t2.start()
        time.sleep(0.175)
    elif state == 1:
        print("State 1")
        state =2
        pass
    else:
        t2.join()
        print("halt")
        #Change this to be the location we grab from the april tags
        message = "received distress at: " + str(goal_x) + "," + str(goal_z)
        t1 = threading.Thread(target=send, name='t1', args=(message,))
        t1.start()
        state = 2
        waiting = False
        t1.join()
        # arduino.write(struct.pack('>B',4))

#t_app = threading.Thread(target=start_app, name='t_app')

if __name__ == '__main__':
    start_app()