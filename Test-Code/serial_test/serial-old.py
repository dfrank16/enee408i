import serial

port = serial.Serial("/dev/tty.ACM0", 19200)
def portRead():
    w = port.in_waiting
    b = port.read(w)
    print("Read ", w, " bytes from port. Read: ", b)
