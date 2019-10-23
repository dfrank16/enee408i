from flask import Flask, render_template
from flask_ask import Ask, statement, question
import time
import serial
import struct

#ser = serial.Serial('/dev/ttyACM0',9600)

app = Flask(__name__)
ask = Ask(app, '/')


def halt():
	valueToWrite= 0
	#ser.write(struct.pack('>B', valueToWrite))

def left():
	valueToWrite= 1
	#ser.write(struct.pack('>B', valueToWrite))

def right():
	valueToWrite= 2
	#ser.write(struct.pack('>B', valueToWrite))

def forward():
	valueToWrite= 3
	#ser.write(struct.pack('>B', valueToWrite))

def backward():
	valueToWrite= 4
	#ser.write(struct.pack('>B', valueToWrite))
	
def wander():
	valueToWrite= 5
	#ser.write(struct.pack('>B', valueToWrite))



@ask.launch
def launched():
    return question("Hello. what would you like Murphy to do?").reprompt(
        "if you don't need Murphy, please tell him to go to sleep.")


@ask.intent('MoveIntent')
def move(direction):
    msg = ""
    if direction == 'left':
        left()
        time.sleep(1.0)
        halt()
        msg = "Murphy moved left"
    elif direction == 'right':
        right()
        time.sleep(1.0)
        halt()
        msg = "Murphy moved right"
    elif direction == 'forward':
        forward()
        time.sleep(4.0)
        halt()
        msg = "Murphy moved forward"
    elif direction == 'backward':
        backward()
        time.sleep(2.0)
        halt()
        msg = "Murphy moved backward"
    elif direction == 'halt':
        halt()
        msg = "Murphy has stopped moving"
    elif direction == "move":
        return question("In what direction?").reprompt("Can you please give a direction?")	
    return question(msg).reprompt("What would you like Murphy to do now?")


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
    return question("AHHHHHHHHHHHHHHHHHHH.").reprompt("Murphy has calmed down now. What would you like him to do?")


@ask.intent('RollIntent')
def rollOver():    
    right()
    time.sleep(6.0)
    halt()
    return question("Ayyye! Murphy rolled over.").reprompt("What would you like Murphy to do now?")


@ask.intent('AMAZON.FallbackIntent')
def default():
    return question("Sorry, Murphy doesn't understand that command. What would you like him to do?").reprompt(
        "What would you like Murphy to do now?")


@ask.intent('SleepIntent')
def sleep():
    # halt()
    print("WE sleepING boiis")
    return statement('Murphy says he is snoring. Bye!')


if __name__ == '__main__':
    app.run(debug=True)
