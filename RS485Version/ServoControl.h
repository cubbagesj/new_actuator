/*------------------------------------------------------------------
 * ServoControl.h - Include file for Servo Control Program
 *
 * This file contains the needed includes and function prototypes 
 * for the ServoControl program
 * ----------------------------------------------------------------
 */


/*-----------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <syslog.h>

#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>


/*-----------------------------------------------------------------
 * Constants
 * ----------------------------------------------------------------
 */

#define SERVOINPORT      "4953"
#define SERVOOUTPORT     "3187"
#define CONTROLOUTPORT   "4955"
#define CONTROLINPORT    "4950"
#define SERVOIP          "192.168.1.50"
#define CONTROLIP        "192.168.1.8"
#define DATASIZE        735

/*********** Define the packet syncwds **************/
#define SHORE_PKT   0xf0a1
#define OBC_PKT     0xf0c1
#define STATUS_PKT  0xf0b1
#define SAFE_PKT    0xf0ff
#define RECOVER_PKT 0xf0ef
#define ERROR_PKT   0xf001
#define CONFIG_PKT  0xf0d1
#define BMS_PKT     0xe1f0
#define CMD_PKT     0xf0f1

#define PKT_TIMOUT  2
#define INTERNAL    0
#define EXTERNAL    1

/************** Data Arrays **********************/
unsigned char buff_in[2*DATASIZE -1];

/*-----------------------------------------------------------------
 * Structure Definitions
 *-----------------------------------------------------------------
 */
struct control_msg
{
    unsigned short  syncwd;
    short           servocmd;
    short           encoder_ang;
    short           motor_ang;
};


struct servo_cmd
{
    unsigned char   header[4];
    int             value;
    unsigned char   chksum;
};

struct servo_msg
{
    char            address;
    char            module;
    char            status;
    char            cmdnum;
    int             value;
    unsigned char   chksum;
};




/*-----------------------------------------------------------------
 * Function Headers
 * ----------------------------------------------------------------
 */

short passiveUDP(const char *service);
short connectUDP(char const *host, char const *service);
short errexit(const char *format, ...);
int servo_move(float);
float servo_read(int);

#endif
