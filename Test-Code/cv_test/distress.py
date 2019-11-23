 
# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from collections import deque
# from imutils.video import VideoStream
import numpy as np
import argparse
# import imutils
import time
import struct
from flask import Flask, render_template
#from flask_ask import Ask
# import serial
# arduino = serial.Serial('/dev/ttyACM0', 9600)

state = 0
goal_x = 0.0
goal_z = 0.0

import socket
import sys
import errno
import threading
HEADER_LENGTH = 10
IP = "192.168.43.59"
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



waiting = True
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
