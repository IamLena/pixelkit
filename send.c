#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

int main (int argc, char **argv) {
	char *portname;
	if (argc == 2)
		portname = argv[1];
	else
		portname = "/dev/ttyACM0";
	printf("writing to %s\n", portname);
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		printf ("error %d opening %s: %s", errno, portname, strerror(errno));
		return 1;
	}
	printf("fd = %d\n", fd);
	printf("openned; sending msg\n");
	int res = write (fd, "hello!\n", 7);				// send 7 character greeting
	printf("res = %d\n", res);
	usleep ((7 + 25) * 100); // sleep enough to transmit the 7 plus receive 25: approx 100 uS per char
	printf("message sent\n");
	close(fd);
	printf("closed\n");
	return 0;
}
