import serial
import time

port = serial.Serial("/dev/tty.usbmodem14201", 19200)
def portRead():
    w = port.in_waiting
    b = port.read(w)
    if w > 0:
        print "Read ", w, " bytes from port. Read: ", b
    else:
        print 0

count = 32;

while 1:
    port.write(str(count))
    time.sleep(1)
    portRead()
    count += 1
    if count == 255:
        count = 32
