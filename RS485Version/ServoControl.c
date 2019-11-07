/* ------------------------------------------------------------
 * ServoControl.c - Autonomous Model Servo Control Program
 *
 * This is the control program for the new Autonomous
 * Model servo.  It gets command messages from control and
 * sends them to the servo and also reads the servo position
 * and sends it back to Control
 *
 * Written By: Samuel J. Cubbage
 *
 * -------------------------------------------------------------
 */

/*--------------------------------------------------------------
 *  General Includes
 *--------------------------------------------------------------
 */

#include <sys/time.h>


/*-------------------------------------------------------------
 *  Autonomous Model Includes 
 *  ServoControl.h - Defines the data structures and function headers
 * ------------------------------------------------------------
 */

#include "ServoControl.h"

int main(void)
{
    /*----------------------------------------------------------------
     * The main loop waits for the arrival of a command packet
     * When it arrives it then replies with the current servo position
     * data and then sends the command to the servo.  Finally it reads
     * the servo position and stores it to send on the next time through
     * ---------------------------------------------------------------
     */

    /**************Initialize Some Program Variables ****************/
    unsigned short  syncwd = 0;

    /* Initialize the servo positions */ 
    float encoder_angle = 0.0;
    float motor_angle = 0.0;

    char shore_buf[179];
    
    struct control_msg  commands;

    /* Declarations for the sockets */

    char *control_host = CONTROLIP;          /* address of Control PC */
    char *in_service = CONTROLOUTPORT;       /* Port to listen to Control on */
    char *out_service = CONTROLINPORT;             /* Port to talk to Control on */

    int s_in;            /* Input socket descriptors */
    int s_control;         /* Output socket descriptors */
    unsigned int alen;
    char *nic0 = "eth0";

    int readtoggle = 0;
    

    /* Declarations for Select Call */
    struct timeval tv;          /* Timeout for recvfrom call */
    fd_set readfds, afds;

    tv.tv_sec = PKT_TIMOUT;         /* Set OBC timeout value */
    tv.tv_usec = 0;

    /* Other declarations */
    struct sockaddr_in fsin;        /* the from address of a client */
    short pkt_count = 0, i = 0;
    short nb;
    
    /*----------------------------------------------------------------
     * End Declarations Section
     * ---------------------------------------------------------------
     */

    /*----------------------------------------------------------------
     * Begin Initialization Section
     * ---------------------------------------------------------------
     */

    /************************************************
     * Initialize the Sockets
     ************************************************
    */

    /* First the listen socket */
    s_in = passiveUDP(in_service);

    /* Then the talk sockets */
    s_control = connectUDP(control_host, out_service);


    /* Now set up the descriptors for select call */
    FD_ZERO(&afds);
    FD_SET(s_in, &afds);

    /* Initialize the positions in the message */
    commands.syncwd = CMD_PKT;
    commands.encoder_ang = (int) (encoder_angle * 100);
    commands.motor_ang = (int) (motor_angle * 100);
    
    /* infinite loop for packets */
    while (1) 
    {

        /* Use the select function to wait until a packet is recvd on s_in
           the value in tv is the timeout to wait for a packet */

        memcpy(&readfds, &afds, sizeof(afds));
        if (select(s_in+1, &readfds, NULL, NULL, &tv) > 0) 
        {
            /* Packet Available */
            tv.tv_sec = PKT_TIMOUT;     /* reset timer */
        
            /* receive packet into buff_in */ 
            nb = recvfrom(s_in, &buff_in[0], (size_t)sizeof(buff_in), 0,
                                               (struct sockaddr *)NULL, &alen);

            /** copy syncwd from buff_in - syncwd it the first 2 bytes in the pkt */
            memcpy(&syncwd,&buff_in[0],2);
        } 
        else 
        {   /* Packet Timeout - No packets in PKT_TIMOUT */

            /* Clear out syncwd so the rest of code does not execute */
            syncwd = 0;     /* clear syncwd */
            tv.tv_sec = PKT_TIMOUT;      /* reset timer */
            printf("Timeout\n");
        }

        /* Based on the syncwd execute the appropriate commands */
        switch (syncwd) 
        {
            case CMD_PKT:         /* Command Packet */

            
                /* First thing is to reply with latest position message */
                write(s_control, &commands, sizeof(commands)); 
		
                /* copy buff_in into control_msg structure 
                 * Only copy command part - ignore angles */
                memcpy(&commands,&buff_in[0], 4);


                /* The controller expects an angle command.  The current value in
                 * commands.serocmd is the angle * 100 so it can be an integer
                 * Need to convert to float angle and then send to servo
                 *
                 * But first, we compare it to the current position and only
                 * send an update if the new command has changed.  This is to
                 * reduce how much the stepper has to work.
                 */

                /* Compare new command with current position 
                  Look for a difference of > 0.1 deg*/

                 /*if (abs(((float) commands.servocmd /100.0) - encoder_angle) > .10) 
                 {
                     servo_move(((float) commands.servocmd) / 100.0);
                 }*/
                 

                 servo_move(((float) commands.servocmd) / 100.0);
                 /* Now get the encoder position */

                 if (readtoggle++ >= 5){
                     readtoggle = 0;
                     encoder_angle = servo_read(INTERNAL);
                     if (encoder_angle >= -900){
                         commands.motor_ang = (short) (encoder_angle * 100);
                     }
                 }
                 else {
                     encoder_angle = servo_read(EXTERNAL);
		             if (encoder_angle >= -900)
                     {
                         /* -900 means a bad response so keep the last value */
                         commands.encoder_ang = (short) (encoder_angle * 100);
		             }
                 }

                 //printf("Cmd: %d, Motor Angle: %f, Enc Angle: %f\n",commands.servocmd, commands.motor_ang/100.0, commands.encoder_ang / 100.0 );
                break;
    
            default:

                    break;
        }/* end switch */
    }/* end while loop */

    return 0;
} /* End of Main */


int servo_move(float angle)
{
    char *servo = SERVOIP;
    char *out_port = SERVOINPORT;
    char *in_port = SERVOOUTPORT;
    int s_servo, s_in;
    char *nic0 = "eth0";

    unsigned char buf[10];
    unsigned int alen;
    int rdlen;

    /* Set up socket */
    s_servo = connectUDP(servo, out_port);
    s_in = passiveUDP(in_port);

    /* Temp command variable to help w/chksum */
    unsigned char move_cmd[9];

    /* Misc variables */
    unsigned char Checksum, i, j;

    struct servo_cmd      move_angle;

    /* Configure the move command */
    move_angle.header[0] = 0x01;
    move_angle.header[1] = 0x04;
    move_angle.header[2] = 0x00;
    move_angle.header[3] = 0x00;
    move_angle.value = htobe32((int) (angle*444.444));

    memcpy(&move_cmd[0], &move_angle, sizeof(move_cmd));

    Checksum = move_cmd[0];
    for (i=1; i<8; i++)
        Checksum += move_cmd[i];
    
    move_angle.chksum = Checksum;

    /* Send the move command */ 
    write(s_servo, &move_angle, 9);

    rdlen = recvfrom(s_in, buf, 9, 0, (struct sockaddr *)NULL, &alen);

    close(s_servo);
    close(s_in);
 
}

float servo_read(int source)
{
    char *servo = SERVOIP;
    char *out_port = SERVOINPORT;
    char *in_port = SERVOOUTPORT;
    int s_servo, s_in;
    char *nic0="eth0";

    short nb;
    unsigned int alen;

    /* Declarations for Select Call */
    struct timeval tv;          /* Timeout for recvfrom call */
    fd_set readfds, afds;

    tv.tv_sec = 0;         /* Set timeout value */
    tv.tv_usec = 20000;

    /* Set up socket */
    s_servo = connectUDP(servo, out_port);
    s_in = passiveUDP(in_port);

    FD_ZERO(&afds);
    FD_SET(s_in, &afds);

    /* Set up command structures */
    struct servo_cmd readenc_cmd;
    struct servo_cmd readmotor_cmd;

    /* Misc variables */
    float angle;

    /* Create a structure for the response message */

    struct servo_msg    reply;

    /* Set up read external encoder sequence */ 
    readenc_cmd.header[0] = 0x01;
    readenc_cmd.header[1] = 0x06;
    readenc_cmd.header[2] = 0xd8;
    readenc_cmd.header[3] = 0x00;
    readenc_cmd.value = 0;
    readenc_cmd.chksum = 0xdf;

    /* Set up read motor sequence */ 
    readmotor_cmd.header[0] = 0x01;
    readmotor_cmd.header[1] = 0x06;
    readmotor_cmd.header[2] = 0x01;
    readmotor_cmd.header[3] = 0x00;
    readmotor_cmd.value = 0;
    readmotor_cmd.chksum = 0x08;


    unsigned char buf[10];

    /* Depending on source send command to read the encoder */
    if (source == EXTERNAL)
    {
        write(s_servo, &readenc_cmd, 9);
    }
    else
    {
        write(s_servo, &readmotor_cmd, 9);
    }

    /* Read the response  - Use select to poll for response*/

    memcpy(&readfds, &afds, sizeof(afds));
    if (select(s_in+1, &readfds, NULL, NULL, &tv) > 0)
    {
        /* Respone available */
        nb = recvfrom(s_in, buf, 9, 0, (struct sockaddr *)NULL, &alen);
        if (nb == 9){
            memcpy(&reply, &buf[0], sizeof(reply));
            angle = (int) htobe32(reply.value)/ 444.444;
        }
        else
        {
            /* Bad packet so send flag */
            angle = -999;
        }
    }
    else
    {
        /* Time out - send flag */
        printf("Timeout\n");
        angle = -999;
    }


    close(s_in);
    close(s_servo);

    return(angle);
}

