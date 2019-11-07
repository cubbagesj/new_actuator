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
    char *portname = "/dev/ttyS0";
    int fd;
    int wlen, rdlen;

    /* variable move command */
    unsigned char move_cmd[9];
    unsigned char buf[10];

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

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0){
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    set_interface_attribs(fd, B115200);

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
    wlen = write(fd, move_cmd, 9);
    if (wlen != 9){
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);

    rdlen = read(fd, buf, sizeof(buf) -1);
    printf("\n%d\n", rdlen);

    for (j=0; j<9; j++)
        printf("%x ",buf[j]);

}

