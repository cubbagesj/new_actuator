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


    /* Misc variables */
    unsigned char Checksum, i, j;


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

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0){
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    set_interface_attribs(fd, B9600);

    /* Send the move command */ 
    wlen = write(fd, home_cmd, 9);
    if (wlen != 9){
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);

}

