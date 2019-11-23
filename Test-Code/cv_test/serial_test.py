
from time import sleep
import serial
ser = serial.Serial('/dev/ttyACM0', 19200) # Establish the connection on a specific port
counter = 32 # Below 32 everything in ASCII is gibberish
ser.reset_input_buffer()
ser.reset_output_buffer()
while True:
     counter +=1
     print(counter)
     ser.write('h'.encode()) # Convert the decimal number to ASCII then send it to the Arduino
     sleep(.1)
     if ser.in_waiting:
         print(ser.readline()) # Read the newest output from the Arduino
     sleep(.1) # Delay for one tenth of a second
     if counter == 255:
         counter = 32
