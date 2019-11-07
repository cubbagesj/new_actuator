# actuatorcmd.py - script to send test commands to the 
# new actuator and controller
#

import sys
import serial
import time
import binascii

conn = serial.Serial()
conn.baudrate = 9600
conn.timeout = 1
conn.port = '/dev/ttyACM0'
conn.open()


# Run initialization routine
#
conn.write(b'\x01\x81\x01\x00\x00\x00\x00\x00\x83')

conn.close()









