# actuatorcmd.py - script to send test commands to the 
# new actuator and controller
#

import sys
import serial
import time
import binascii

conn = serial.Serial()
conn.baudrate = 9600
conn.timeout = .1
conn.port = '/dev/ttyACM0'
conn.open()


conn.write(b'\x01\x04\x00\x00\x00\x00\x08\xae\xbb')

time.sleep(.01)

while False:

    conn.write(b'\x01\x06\xd8\x00\x00\x00\x00\x00\xdf')

    resp = conn.read(9)
    print(int(binascii.b2a_hex(resp)[8:16], base=16)/222.2222)

    time.sleep(.01)



conn.close()









