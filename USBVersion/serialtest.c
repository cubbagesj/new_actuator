#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <endian.h>


int set_interface_attribs( int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    /* setip for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0){
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}


int main(int argc, char *argv[])
{
    char *portname = "/dev/ttyACM0";
    int fd;
    int wlen;
    /* Set up pre-defined commands */
    unsigned char home_cmd[9];
    unsigned char readenc_cmd[9];
    unsigned char readmotor_cmd[9];

    /* variable move command */
    unsigned char move_cmd[9];

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

    /* Create a structure for the move command */
    struct var_cmd
    {
        unsigned char   header[4];
        int             value;
        unsigned char   chksum;
    };

    struct var_cmd      move_angle;

    /* Set up homing sequence */
    home_cmd[0] = 0x01;
    home_cmd[1] = 0x81;
    home_cmd[2] = 0x01;
    home_cmd[3] = 0x00;
    home_cmd[4] = 0x00;
    home_cmd[5] = 0x00;
    home_cmd[6] = 0x00;
    home_cmd[7] = 0x00;
    home_cmd[8] = 0x83;

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


    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0){
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    set_interface_attribs(fd, B9600);

    /* Configure the move command */
    move_angle.header[0] = 0x01;
    move_angle.header[1] = 0x04;
    move_angle.header[2] = 0x00;
    move_angle.header[3] = 0x00;
    move_angle.value = htobe32((int) atof(argv[1])*222.222);

    memcpy(&move_cmd[0], &move_angle, sizeof(move_cmd));

    Checksum = move_cmd[0];
    for (i=1; i<8; i++)
        Checksum += move_cmd[i];
    move_cmd[8] = Checksum;

    for (j=0; j<9; j++)
        printf("%x ",move_cmd[j]);

    /* Send the move command */ 
    wlen = write(fd, move_cmd, 9);
    if (wlen != 9){
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);

    do {
        unsigned char buf[10];
        int rdlen;

        wlen = write(fd, readenc_cmd, 9);
        if (wlen != 9){
            printf("Error from write: %d, %d\n", wlen, errno);
        }
        tcdrain(fd);
        
        rdlen = read(fd, buf, sizeof(buf) -1);
        if (rdlen > 0){
            memcpy(&reply, &buf[0], sizeof(reply));
            ex_angle = htobe32(reply.value)/ 222.2222;
        }

        /* Send the command */
        wlen = write(fd, readmotor_cmd, 9);
        if (wlen != 9){
            printf("Error from write: %d, %d\n", wlen, errno);
        }
        tcdrain(fd);
        
        rdlen = read(fd, buf, sizeof(buf) -1);
        if (rdlen > 0){
            memcpy(&reply, &buf[0], sizeof(reply));
            int_angle = htobe32(reply.value)/222.2222;
        }

        printf("Angles: Motor: %f  External: %f \n", int_angle, ex_angle);
    } while (1);
}

