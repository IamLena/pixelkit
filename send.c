#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

int main(int argc, char **argv)
{
	int fd;
	struct termios term;
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
	fd = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
	if (fd == -1)
		perror("Unable to open port");

	fcntl(fd, F_SETFL, 0); // clear all file status flags

	memset(&term, 0, sizeof(struct termios));
	if ((cfsetispeed(&term, B9600) < 0) ||
		(cfsetospeed(&term, B9600) < 0)) {
			perror("Unable to set baudrate");
	}

	/*
	CREAD - включить прием
	CLOCAL - игнорировать управление линиями с помошью
	CSIZE - маска размера символа
	CS8 - 8 битные символы
	CSTOPB - при 1 - два стоп бита, при 0 - один
	*/
	term.c_cflag |= (CREAD | CLOCAL);
	term.c_cflag &= ~CSIZE;
	term.c_cflag |= CS8;
	term.c_cflag &= ~CSTOPB;

	/*
	ICANON - канонический режим
	ECHO - эхо принятых символов
	ECHOE - удаление предыдущих символов по ERASE, слов по WERASE
	ISIG - реагировать на управляющие символы остановки, выхода, прерывания
	INPCK - вкл. проверку четности
	OPOST - режим вывода по умолчанию
	*/
	term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	term.c_iflag &= ~(INPCK);
	term.c_oflag &= ~OPOST;

	/*
	TCSANOW - примернить изменения сейчас же
	TCSADRAIN - применить после передачи всех текущих данных
	TCSAFLUSH - приемнить после окончания передачи, все принятые но не считанные данные очистить
	*/
	if (tcsetattr(fd, TCSANOW, &term) < 0)
		perror("Unable to set port parameters");

	writeres = write(fd, message, strlen(message));
	if (writeres == -1)
		printf("write return -1\n");
	else
		printf("%d bytes are sent\n", writeres);
	close(fd);
}
