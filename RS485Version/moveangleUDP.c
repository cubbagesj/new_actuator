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
    int s_servo;
    char *nic0 = "eth0";

    /* Set up socket */
    s_servo = connectUDP(servo, out_service);


    /* variable move command */
    unsigned char move_cmd[9];

    /* Misc variables */
    unsigned char Checksum, i, j;


    /* Create a structure for the move command */
    struct var_cmd
    {
        unsigned char   header[4];
        int             value;
        unsigned char   chksum;
    };

    struct var_cmd      move_angle;


    /* Configure the move command */
    move_angle.header[0] = 0x01;
    move_angle.header[1] = 0x04;
    move_angle.header[2] = 0x00;
    move_angle.header[3] = 0x00;
    move_angle.value = htobe32((int) (atof(argv[1])*222.222));

    memcpy(&move_cmd[0], &move_angle, sizeof(move_cmd));

    Checksum = move_cmd[0];
    for (i=1; i<8; i++)
        Checksum += move_cmd[i];
    move_cmd[8] = Checksum;

    for (j=0; j<9; j++)
        printf("%x ",move_cmd[j]);
    

    /* Send the move command */ 
    write(s_servo, move_cmd, 9);

}

