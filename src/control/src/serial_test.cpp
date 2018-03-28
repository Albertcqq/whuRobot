
#include <math.h>
#include <stdio.h>      // standard input / output functions
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitionss


int openPort(const char * dev_name){
    int fd; // file description for the serial port
    fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1){ // if open is unsucessful
        printf("open_port: Unable to open /dev/ttyS0. \n");
    }
    else  {
        fcntl(fd, F_SETFL, 0);
        printf("port is open.\n");
    }
    return(fd);
}

int configurePort(int fd){                      // configure the port
    struct termios port_settings;               // structure to store the port settings in
    cfsetispeed(&port_settings, B115200);       // set baud rates
    cfsetospeed(&port_settings, B115200);

    port_settings.c_cflag &= ~PARENB;           // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port
    return(fd);
}

bool sendData(int fd, short int mode,short int orientation,short int v,short int yaw,short int pitch,short int dx,short int dy){
    unsigned char send_bytes[] = { 0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E};
    short * data_ptr = (short *)(send_bytes + 1);
    data_ptr[0] = (short)mode;
    data_ptr[1] = (short)orientation;
    data_ptr[2] = (short)v;
    data_ptr[3] = (short)yaw;
    data_ptr[4] = (short)pitch;
    data_ptr[5] = (short)dx;
    data_ptr[6] = (short)dy;
	printf ("%X %X%X %X%X %X%X %X%X %X%X %X%X %X%X %X\n",send_bytes[0],send_bytes[1],send_bytes[2],send_bytes[3],send_bytes[4],send_bytes[5 ],send_bytes[6],send_bytes[7],send_bytes[8],send_bytes[9],send_bytes[10],send_bytes[11],send_bytes[12],send_bytes[13],send_bytes[14],send_bytes[15]);
    if (16 == write(fd, send_bytes,16))      //Send data
        return true;
printf("error\n");
    return false;
}

bool send(int fd)
{
unsigned char str[]={0x7F,0x00,0x7E};
if (3 == write(fd, str,3))      //Send data
        return true;
}


int main()
{
    int fd=openPort("/dev/ttyUSB0");
    configurePort(fd);
    while(1)
    {
        sendData(fd,1,0,0,10,10,3550,30);
        
//send(fd);
usleep(100000);
    }
}
