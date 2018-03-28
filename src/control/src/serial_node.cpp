
#include <math.h>
#include <stdio.h>      // standard input / output functions
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitionss
#include "serial/serial.h"
#include <signal.h>
int mode;
static struct sigaction siga;

static void callback(int sig,siginfo_t * siginfo,void* context)
{
    if (sig==SIGINT)
    {

        mode=0;
        printf("STOP\n");
    }
}

int main()
{
    mode=1;
    siga.sa_sigaction=*callback;
    siga.sa_flags|=SA_SIGINFO;
    if (sigaction(SIGINT,&siga,NULL)!=0)
    {
        printf("Error sig\n");
    }
    int fd=openPort("/dev/ttyUSB0");
    configurePort(fd);
    struct robot_message_out msg;
    msg.mode=1;
    msg.yaw=10;
    msg.pitch=20;
    msg.dx=32;
    msg.dy=-35;
    while(mode)
    {
        sendData(fd,msg);
        usleep(100);
    }
    close(fd);
}
