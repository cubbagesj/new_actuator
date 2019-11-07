import sys
import serial
import time
import binascii

conn = serial.Serial()
conn.baudrate = 9600
conn.timeout = 1
conn.port = '/dev/ttyACM0'
conn.open()


ang = 222.2222 * float(sys.argv[1])

# Run initialization routine
#
conn.write(b'\x01\x81\x01\x00\x00\x00\x00\x00\x83')

time.sleep(1)


init_ascii = b'\x01\x8B\x00\x00\x00\x00\x00\x00\x8C'

conn.write(init_ascii)
time.sleep(1)

conn.write('ASAP, 4, 0, 996\r')
time.sleep(0.02)

resp = conn.read(20)

print(str(len(resp)) + '\r')
print(resp)

conn.write('AMVP ABS, 0, ' + str(ang) + '\r')
time.sleep(0.02)

resp = conn.read(20)

print(str(len(resp)) + '\r')
print(resp)

conn.close()









