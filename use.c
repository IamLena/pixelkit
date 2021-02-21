#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
# define CRTSCTS  020000000000 // can't find itd definition in headers

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	if (tcgetattr (fd, &tty) != 0)
	{
		printf("error %d from tcgetattr\n", errno);
		return -1;
	}
	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	// 8-bit chars
	tty.c_iflag &= ~IGNBRK;						// disable break processing
	tty.c_lflag = 0;							// no signaling chars, no echo,
	tty.c_oflag = 0;							// no remapping, no delays
	tty.c_cc[VMIN]  = 0;						// read doesn't block
	tty.c_cc[VTIME] = 5;						// 0.5 seconds read timeout
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);		// shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);			// ignore modem controls, enable reading
	tty.c_cflag &= ~(PARENB | PARODD);			// shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf ("error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tggetattr", errno);
		return;
	}
	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;	// 0.5 seconds read timeout
	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf ("error %d setting term attributes", errno);
}

int main (void) {
	char *portname = "/dev/ttyACM0";
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		printf ("error %d opening %s: %s", errno, portname, strerror(errno));
		return 1;
	}
	printf("openned\n");

	// set_interface_attribs(fd, B115200, 0);	// set speed to 115,200 bps, 8n1 (no parity)
	// set_blocking(fd, 0);					// set no blocking

	printf("sending msg\n");
	write (fd, "hello!\n", 7);				// send 7 character greeting
	usleep ((7 + 25) * 100); // sleep enough to transmit the 7 plus receive 25: approx 100 uS per char
	close(fd);
	printf("msg sent, fd closed\n");
	fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		printf ("error %d opening %s: %s", errno, portname, strerror(errno));
		return 1;
	}
	printf("openned\n");
	char buf [100];
	printf("reading\n");
	int n = read (fd, buf, sizeof buf);
	printf("n = %d\n", n);
	if (n != -1)
	{
		buf[n] = '\0';
		printf("buf %s\n", buf);
	}
	close (fd);
	printf("closed\n");
	return 0;
}
