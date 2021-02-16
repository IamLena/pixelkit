#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

int main(void)
{
	int fd;
	// char *name = "/dev/ttyACM0";
	char *name = "/dev/skel0";
	fd = open(name, O_RDWR | O_SYNC | O_NOCTTY);
	if (fd == -1)
		printf("fd = -1, error %s\n %d", strerror(errno), errno);
	else {
		int output = write(fd, "hello", 5);
		printf("write output = %d\n", output);
		close (fd);
		printf("file is closed\n");
	}

	fd = open(name, O_RDWR | O_SYNC | O_NOCTTY);
	if (fd == -1)
		printf("fd = -1, error %s\n %d", strerror(errno), errno);
	else {
		char buf[6];
		int output = read(fd, buf, 5);
		printf("read output = %d\n", output);
		if (output != -1) {
			buf[output] = '\0';
			printf("data: %s\n", buf);
		}
		close (fd);
		printf("file is closed\n");
	}
}
