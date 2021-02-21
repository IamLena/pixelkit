#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

int main (void) {
	char *portname = "/dev/ttyACM0";
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		printf ("error %d opening %s: %s", errno, portname, strerror(errno));
		return 1;
	}
	printf("openned\n");
	printf("sending msg\n");
	write (fd, "hello!\n", 7);				// send 7 character greeting
	usleep ((7 + 25) * 100); // sleep enough to transmit the 7 plus receive 25: approx 100 uS per char
	close(fd);
	printf("closed\n");
	return 0;
}
