#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

char *tx = "hello world!";

int main(int argc, char **argv)
{
    printf("usage: ./senddata [message [port_path]]\n");
    char *message = "";
    char *portPath = "/dev/ttyACM0";
    if (argc == 2 || argc == 3)
        message = argv[1];
    if (argc == 3)
        portPath = argv[2];

    int fd;
    fd_set fs;
    struct timeval timeout;
    struct termios term;
    char buf[512];
    int res;
    int i;

    /*
    O_RDWR - чтение и запись
    O_NONBLOCK - не блокировать файл
    O_NOCTTY - не делать устройство управлящим терминалом
    O_NDELAY - не использовать DCD линию
    */
    fd = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open port");
    }

    fcntl(fd, F_SETFL, 0); // clear all file status flags

    memset(&term, 0, sizeof(struct termios));

    if ((cfsetispeed(&term, B9600) < 0) ||
        (cfsetospeed(&term, B9600) < 0)) {
            perror("Unable to set baudrate");
    }

    /*
    CREAD - включить прием
    CLOCAL - игнорировать управление линиями с помошью
    */
    term.c_cflag |= (CREAD | CLOCAL);
    /* CSIZE - маска размера символа */
    term.c_cflag &= ~CSIZE;
    /* CS8 - 8 битные символы */
    term.c_cflag |= CS8;
    /* CSTOPB - при 1 - два стоп бита, при 0 - один */
    term.c_cflag &= ~CSTOPB;

    /*
    ICANON - канонический режим
    ECHO - эхо принятых символов
    ECHOE - удаление предыдущих символов по ERASE, слов по WERASE
    ISIG - реагировать на управляющие символы остановки, выхода, прерывания
    */
    term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /* INPCK - вкл. проверку четности */
    term.c_iflag &= ~(INPCK);
    /* OPOST - режим вывода по умолчанию */
    term.c_oflag &= ~OPOST;

    /*
    TCSANOW - примернить изменения сейчас же
    TCSADRAIN - применить после передачи всех текущих данных
    TCSAFLUSH - приемнить после окончания передачи, все принятые но не считанные данные очистить
    */
    if (tcsetattr(fd, TCSANOW, &term) < 0)
    {
        perror("Unable to set port parameters");
    }

    write(fd, tx, 8);

    timeout.tv_sec  = 2;
    timeout.tv_usec = 0;
    FD_ZERO (&fs);
    FD_SET(fd, &fs);

    // res = select ( fd+1 , &fs, NULL, NULL, &timeout );

    // if (res == 0) {
    //     perror("Timeout occurs");
    // } else if (res > 0) {
    //     if ( FD_ISSET(fd, &fs) ) {
    //         memset(buf,0x00,sizeof(buf));
    //         res = read(fd,buf,512);
    //         printf("Read %d bytes:\n", res);
    //         for (i=0;i<res;i++)
    //             printf("%c ", buf[i]);
    //         printf("\n");
    //     }
    // }

    // to clode fd ?

}
