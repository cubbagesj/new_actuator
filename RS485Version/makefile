#  Makefile to make and compile the moveangle program
#
#
#
cobjects = ServoControl.o passivesock.o connectUDP.o \
           errexit.o connectsock.o passiveUDP.o 

#Set directory search path
VPATH = C_code
		   
CC = gcc

# C programs need -fpack-struct so they don't pad the structures
CFLAGS = -fpack-struct


#moveangle:  $(cobjects)  
#	gcc -o moveangleUDP  $(cobjects)


readposition:  $(cobjects)  
	gcc -o ServoControl  $(cobjects)



$(cobjects): %.o: %.c ServoControl.h
	$(CC) -c $(CFLAGS) $< -o $@

