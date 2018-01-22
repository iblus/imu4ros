#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>

#include "uart.h"

static int const baudRateTable[] = {B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
static int const baudRateMap[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};

/****************************************************************
Return value:	 	None
****************************************************************/
static void setup_port(int fd, int baudRate, int dataBits, int parity, int stopBits)
{
    int i;
    struct termios opt;

    for (i = 0; i < sizeof(baudRateMap) / sizeof(baudRateMap[0]); i++)
    {
        if (baudRate == baudRateMap[i])
            baudRate = baudRateTable[i];
    }

    tcgetattr(fd, &opt);
    tcflush(fd, TCIOFLUSH);

    cfsetispeed(&opt, baudRate);
    cfsetospeed(&opt, baudRate);

    switch (parity)
    {
    case 1: // Odd parity
        opt.c_cflag |= (PARENB | PARODD);
        break;

    case 2: // Even parity
        opt.c_cflag |= PARENB;
        opt.c_cflag &= ~(PARODD);
        break;

    case 0: // None parity
    default:
        opt.c_cflag &= ~(PARENB);
        break;
    }
    //opt.c_cflag &= ~PARENB;

    // Set stopbits
    switch (stopBits)
    {
    case 2: // 2 stopbits
        opt.c_cflag |= CSTOPB;
        break;

    case 1: // 1 stopbits
    default:
        opt.c_cflag &= ~CSTOPB;
        break;
    }
    //opt.c_cflag &= ~CSTOPB;

    opt.c_cflag &= ~CSIZE;
    switch (dataBits)
    {
    case 5:
        opt.c_cflag |= CS5;
        break;

    case 6:
        opt.c_cflag |= CS6;
        break;

    case 7:
        opt.c_cflag |= CS7;
        break;

    case 8:
    default:
        opt.c_cflag |= CS8;
        break;
    }
    //opt.c_cflag |= CS8;

    opt.c_cflag &= ~CRTSCTS;                                 //no flow control
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN); //raw mode, not canonical mode
    opt.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET);
    opt.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | PARMRK | INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IUCLC | IXON | IXOFF | IXANY);

    opt.c_cc[VTIME] = 0;
    opt.c_cc[VMIN] = 0;

    tcsetattr(fd, TCSANOW, &opt);
    tcflush(fd, TCIOFLUSH); //Flushes the input and/or output queue

    return;
}
static void setup2_port(int fd, int baudRate, int dataBits, int parity, int stopBits)
{
    int i;
    struct termios opt;

    for (i = 0; i < sizeof(baudRateMap) / sizeof(baudRateMap[0]); i++)
    {
        if (baudRate == baudRateMap[i])
            baudRate = baudRateTable[i];
    }

    if (tcgetattr(fd, &opt) != 0)
    {
        printf("tcgetattr err");
    }
    tcflush(fd, TCIOFLUSH);

    cfsetispeed(&opt, baudRate);
    cfsetospeed(&opt, baudRate);

    tcsetattr(fd, TCSANOW, &opt);

    opt.c_cflag &= ~CSIZE;
    switch (dataBits)
    {
    case 5:
        opt.c_cflag |= CS5;
        break;

    case 6:
        opt.c_cflag |= CS6;
        break;

    case 7:
        opt.c_cflag |= CS7;
        break;

    case 8:
    default:
        opt.c_cflag |= CS8;
        break;
    }

    opt.c_cflag |= CLOCAL | CREAD;

    switch (parity)
    {
    case 1: // Odd parity
        opt.c_cflag |= (PARENB | PARODD);
        break;

    case 2: // Even parity
        opt.c_cflag |= PARENB;
        opt.c_cflag &= ~(PARODD);
        break;

    case 0: // None parity
    default:
        opt.c_cflag &= ~(PARENB | PARODD);
        break;
    }
    opt.c_cflag &= ~CRTSCTS;

    // Set stopbits
    switch (stopBits)
    {
    case 2: // 2 stopbits
        opt.c_cflag |= CSTOPB;
        break;

    case 1: // 1 stopbits
    default:
        opt.c_cflag &= ~CSTOPB;
        break;
    }
    tcsetattr(fd, TCSANOW, &opt);

    opt.c_cflag |= CLOCAL;
    opt.c_cflag |= CREAD;
    tcsetattr(fd, TCSANOW, &opt);

    opt.c_iflag |= IGNPAR;
    opt.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN | ECHOE);
    opt.c_iflag &= ~INPCK;
    opt.c_iflag &= ~ICRNL;
    opt.c_iflag &= ~INLCR;
    opt.c_iflag &= ~ISTRIP;
    opt.c_iflag &= ~IXON;
    opt.c_iflag &= ~BRKINT;

    opt.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &opt))
    {
        printf("tcsetattr() failed\n");
    }
    tcflush(fd, TCIOFLUSH); //Flushes the input and/or output queue

    return;
}
/****************************************************************
Return value:	 	positive value	--	Successful
					negative value	--	Failed
****************************************************************/
int uart_init(char *deviceName, int baudRate, int dataBits, int parity, int stopBits)
{
    int uartFd;

    if ((uartFd = open(deviceName, O_RDWR | O_NOCTTY | O_NDELAY)) <= 0)
    {
        printf("%s--%d, open %s failed!\n", __FILE__, __LINE__, deviceName);
        return -1;
    }
    else
    {
        //tcflush(uartFd, TCIOFLUSH);
        setup_port(uartFd, baudRate, dataBits, parity, stopBits);
    }

    return uartFd;
}
