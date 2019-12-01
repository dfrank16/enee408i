import serial
import time
import struct
port = serial.Serial("/dev/tty.usbmodem14201", 19200)
def portRead():
    w = port.in_waiting
    b = port.read(w)
    if w > 0:
        print "Read ", w, " bytes from port. Read: ", b
    else:
        print 0

#count = 32;

"""
while 1:
    port.write(str(count))
    time.sleep(1)
    portRead()
    count += 1
    if count == 255:
        count = 32=

from time import sleep
#import serial
import struct 
"""
ser = serial.Serial('/dev/ttyACM0', 9600) # Establish the connection on a specific port
counter = 32 # Below 32 everything in ASCII is gibberish
ser.reset_input_buffer()
ser.reset_output_buffer()
while True:
     counter +=1
     ser.write(struct.pack('>B',1)) # Convert the decimal number to ASCII then send it to the Arduino
     sleep(.1)
     if ser.in_waiting:
         print(ser.readline()) # Read the newest output from the Arduino
     sleep(.1) # Delay for one tenth of a second
     if counter == 255:
         counter = 32
