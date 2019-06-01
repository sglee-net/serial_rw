#include <iostream>
//#include <libserial/SerialPort.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

using namespace std;

int main() {
	cout<<"hello"<<endl;

	int fd = open("/dev/ttyUSB0", O_RDWR ); //| 0_NOCTTY);
	if(fd < 0) {
		cout << "error " << endl;
	}

	struct termios tty;
	struct termios tty_old;
	memset(&tty, 0, sizeof tty);

	if(tcgetattr(fd, &tty) != 0) {
		cout<< "error from tcgetattr" <<endl;
	}

	tty_old = tty;

	cfsetospeed(&tty, B9600);
	cfsetispeed(&tty, B9600);

	tty.c_cflag &= -PARENB;
	tty.c_cflag &= -CSTOPB;
	tty.c_cflag &= -CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= -CRTSCTS;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 5;
	tty.c_cflag |= CREAD | CLOCAL;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty.c_oflag &= ~OPOST;

	/* Make raw */
	cfmakeraw(&tty);

	tcflush(fd, TCIFLUSH);

	if(tcsetattr(fd, TCSANOW, &tty) != 0) {
		cout << "error from tcsetattr" << endl;
	}

	
	unsigned char cmd[] = "INIT \r";
	int n_written = 0;
	n_written = write(fd, cmd, sizeof(cmd)-1);

	
	close(fd);
	return 0;
}
