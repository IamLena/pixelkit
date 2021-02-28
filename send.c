#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

int main(int argc, char **argv)
{
	int fd;
	int writeres;

	printf("usage: ./senddata [message [port_path]]\n");
	char *message = "";
	char *portPath = "/dev/ttyACM0";
	if (argc == 2 || argc == 3)
		message = argv[1];
	if (argc == 3)
		portPath = argv[2];

	/*
	O_RDWR - чтение и запись
	O_NONBLOCK - не блокировать файл
	O_NOCTTY - не делать устройство управлящим терминалом
	O_NDELAY - не использовать DCD линию
	*/
	fd = open(portPath, O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
	if (fd == -1)
		perror("Unable to open port");

	fcntl(fd, F_SETFL, 0); // clear all file status flags
	writeres = write(fd, message, strlen(message));
	if (writeres == -1)
		printf("write return -1\n");
	else
		printf("%d bytes are sent\n", writeres);
	close(fd);
}
