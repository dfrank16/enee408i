from flask import Flask, render_template
from flask_ask import Ask, statement, question, audio, current_stream, logger
import collections
from copy import copy
import logging
from collections import deque
import argparse
import sys
import errno
import time

app = Flask(__name__)
ask = Ask(app, '/')
logging.getLogger('flask_ask').setLevel(logging.INFO)

def start_app():
    app.run(debug=True)

@ask.launch
def launched():
	return question("Hello. what would you like Murphy to do?").reprompt("if you don't need Murphy, please tell him to go to sleep.")

@ask.intent('PlayMusicIntent')
def playMusic():
	speech = "Here's one of my favorites"
	#stream_url = 'https://www.vintagecomputermusic.com/mp3/s2t9_Computer_Speech_Demonstration.mp3'
	#stream_url = 'https://archive.org/download/mailboxbadgerdrumsamplesvolume2/Ringing.mp3'
	stream_url = 'https://github.com/dfrank16/enee408i/blob/master/Test-Code/Yeet.mp3'
	return audio(speech).play(stream_url)

@ask.intent('NavigateIntent')
def navigate():
	global navigateFlag
	navigateFlag = 1
	thread.start_new_thread(navigateAprilTag, ())

@ask.intent('ArrivedIntent')
def arrived():
	global navigateFlag
	navigateFlag = 0
	#halt()   # Use this in the actual code, it's just commented out for this test code

def navigateAprilTag():
	global navigateFlag
	while navigateFlag:
		# Insert code here

@ask.intent('MoveIntent')
def move(direction):
	msg = ""
	if direction == 'left':
		time.sleep(1.5)
		msg = "Murphy slid to the left"
	elif direction == 'right':
		time.sleep(1.5)
		msg = "Murphy rides to the rightt"
	elif direction == 'forward':
		time.sleep(2.0)
		msg = "Murphy moved forward boys."
	elif direction == 'backward':
		time.sleep(2.0)
		msg = "Murphy backs the heck up"
	elif direction == 'halt':
		msg = "Murphy has had enough of your hecking crap"
	elif direction == "move":
		return question("In what fooking direction?").reprompt("Can you please give a fooking direction?")
	return question(msg).reprompt("What would you like Murphy to do now?")


@ask.intent('AMAZON.FallbackIntent')
def default():
    return question("Sorry, Murphy doesn't understand that command. What would you like him to do?").reprompt("What would you like Murphy to do now?")


if __name__ == '__main__':
        print("starting app")
        start_app()



