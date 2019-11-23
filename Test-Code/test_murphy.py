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


ser = serial.Serial('/dev/ttyACM0',9600)

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
heading_offset = 10.0
goal_offset = 0.5
z_pos = 0.0
z_neg = 180.0
x_pos = 90.0
x_neg = -90.0

HEADER_LENGTH = 10
IP = "192.168.43.59"
PORT = 1234
my_username = "Murphy"
print("creating socket")
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
username = my_username.encode('utf-8')
username_header = f"{len(username):<{HEADER_LENGTH}}".encode('utf-8')
#client_socket.send(username_header + username)
print("done with socket")


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

@ask.launch
def launched():
    return question("Hello. what would you like Murphy to do?").reprompt(
        "if you don't need Murphy, please tell him to go to sleep.")

@ask.intent('PlayMusicIntent')
def playMusic():
    speech = "Here's one of my favorites"
    stream_url = 'https://www.vintagecomputermusic.com/mp3/s2t9_Computer_Speech_Demonstration.mp3'
    return audio(speech).play(stream_url)

@ask.intent('StopSoundIntent')
def stopSound():
    speech = "Oopsie daisy. I'm sorry master"
    return audio(speech).stop()

@ask.intent('LifeAlertIntent')
def lifeAlert():
    speech = ""
    stream_url = './LifeAlert.mp3'
    return audio(speech).play(stream_url)

@ask.intent('RandyOrtonIntent')
def randyOrton():
    speech = ""
    stream_url = './WatchOutWatchOut.mp3'
    return audio(speech).play(stream_url)

@ask.intent('JohnCenaIntent')
def johnCena():
    speech = ""
    stream_url = './AndHisNameIsJohnCena.mp3'
    return audio(speech).play(stream_url)

@ask.intent('YeetIntent')
def yeet():
    speech = ""
    stream_url = './Yeet.mp3'
    return audio(speech).play(stream_url)

@ask.intent('AhhhIntent')
def ahhh():
    speech = ""
    stream_url = './Ahhh.mp3'
    return audio(speech).play(stream_url)

@ask.intent('CPRIntent')
def cpr():
    speech = ""
    stream_url = './StayingAlive.mp3'
    return audio(speech).play(stream_url, offset=1000)

@ask.intent('PirateMusicIntent')
def pirateMusic():
    speech = ""
    stream_url = './PirateMusic.mp3'
    return audio(speech).play(stream_url)

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
	halt()
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
    return question("Perkele!").reprompt("Murphy has calmed down now. What would you like him to do?")


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
                message_header = f"{len(message):<{HEADER_LENGTH}}".encode('utf-8')
                client_socket.send(message_header + message)      
            except socket.error:          
                time.sleep( 2 ) 
        return

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
                client_socket = socket.socket()
                client_socket.connect((IP, PORT))
                client_socket.setblocking(False)
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
                sys.exit()
            # We just did not receive anything
            continue
        except Exception as e:
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
                except socket.error:          
                    time.sleep( 2 )  

def goto(goal_x, goal_z):
    global z_pos
    global x_neg
    global x_pos
    global curr_heading
    global curr_z
    global curr_x
    goal_theta = z_pos
    while(abs(curr_heading-goal_theta) > heading_offset):
        right()
        time.sleep(0.15)
        getposition()
        halt()
    if curr_z > goal_z:
        while(curr_z-goal_z > goal_offset):
            forward()
            time.sleep(0.15)
            getposition()
            halt()
    else:
        while(goal_z-curr_z > goal_offset):
            backward()
            time.sleep(0.15)
            getposition()
            halt()
    goal_theta = x_pos if goal_x > curr_x else x_neg
    while(abs(curr_heading-goal_theta) > heading_offset):
        right()
        time.sleep(0.15)
        getposition()
        halt()
    if curr_x > goal_x:
        while(curr_x-goal_x > goal_offset):
            forward()
            time.sleep(0.15)
            getposition()
            halt()
    else:
        while(goal_x-curr_x > goal_offset):
            forward()
            time.sleep(0.15)
            getposition()
            halt()
    
    halt()
def findDistress():
    wander()
    while(not waiting):
        time.sleep(0.5)
    goto(goal_x, goal_z)
    return



if __name__ == '__main__':
	print("starting app")
	start_app()
