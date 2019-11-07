#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <endian.h>

#include "AM_Control.h"


int main(int argc, char *argv[])
{
    /* Declaration for socket */
    char *servo = "192.168.1.50";
    char *out_service = OUTPORT2;
    char *in_service = INPORT3;
    int s_servo, s_in;
    char *nic0 = "eth0";
    short nb;
    unsigned int alen;

    /* Set up Socket */
    s_servo = connectUDP(servo, out_service);
    s_in = passiveUDP(in_service);

    /* Set up pre-defined commands */
    unsigned char readenc_cmd[9];
    unsigned char readmotor_cmd[9];

    /* Misc variables */
    unsigned char Checksum, i, j;
    float int_angle, ex_angle;

    /* Create a structure for the response message */
    struct reply_message
    {
        char            address;
        char            module;
        char            status;
        char            cmdnum;
        int             value;
        unsigned char   chksum;
    };

    struct reply_message    reply;

    /* Set up read external encoder sequence */ 
    readenc_cmd[0] = 0x01;
    readenc_cmd[1] = 0x06;
    readenc_cmd[2] = 0xd8;
    readenc_cmd[3] = 0x00;
    readenc_cmd[4] = 0x00;
    readenc_cmd[5] = 0x00;
    readenc_cmd[6] = 0x00;
    readenc_cmd[7] = 0x00;
    readenc_cmd[8] = 0xdf;

    /* Set up read motor sequence */ 
    readmotor_cmd[0] = 0x01;
    readmotor_cmd[1] = 0x06;
    readmotor_cmd[2] = 0x01;
    readmotor_cmd[3] = 0x00;
    readmotor_cmd[4] = 0x00;
    readmotor_cmd[5] = 0x00;
    readmotor_cmd[6] = 0x00;
    readmotor_cmd[7] = 0x00;
    readmotor_cmd[8] = 0x08;


//    do {
        unsigned char buf[10];

        write(s_servo, readenc_cmd, 9);
        
        nb = recvfrom(s_in, buf, sizeof(buf) -1, 0, (struct sockaddr *)NULL, &alen);
        if (nb > 0){
            memcpy(&reply, &buf[0], sizeof(reply));
            ex_angle = (int) htobe32(reply.value)/ 222.2222;
        }

        /* Send the command */
        write(s_servo, readmotor_cmd, 9);
        
        nb = recvfrom(s_in, buf, sizeof(buf) -1, 0, (struct sockaddr *)NULL, &alen);
        if (nb > 0){
            memcpy(&reply, &buf[0], sizeof(reply));
            int_angle = (int) htobe32(reply.value)/222.2222;
        }

        printf("Angles: Motor: %f  External: %f \n", int_angle, ex_angle);
//    } while (1);
}

