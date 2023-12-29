#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstring>

using std::cout; using std::endl;
using std::string; using std::cin;
int power_on_usb(void)
{
    return(system("sudo /home/pi/cams/SonyTEST32/uhubctl/uhubctl -l 1-1 -a 1"));
}

int power_off_usb(void)
{
    return(system("sudo /home/pi/cams/SonyTEST32/uhubctl/uhubctl -l 1-1 -a 0"));
}

void printWaitStatus(const char *msg, int status)
{
    if (msg != nullptr)
        printf("%s", msg);

    if (WIFEXITED(status)) {
        printf("child exited, status=%d\n", WEXITSTATUS(status));
    } else if (WIFSIGNALED(status)) {
        printf("child killed by signal %d (%s)",
               WTERMSIG(status), strsignal(WTERMSIG(status)));
#ifdef WCOREDUMP
        if (WCOREDUMP(status))
            printf(" (core dumped)");
#endif
        printf("\n");
    } else if (WIFSTOPPED(status)) {
        printf("child stopped by signal %d (%s)\n",
               WSTOPSIG(status), strsignal(WSTOPSIG(status)));
#ifdef WIFCONTINUED
    } else if (WIFCONTINUED(status)) {
        printf("child continued\n");
#endif
    } else {
        printf("status=%x\n",
               (unsigned int) status);
    }
}
int main(void)
{
    int ret = power_off_usb();
    if (ret == -1)
        cout << "a child process could not be created, or"
                "its status could not be retrieved!" << endl;
    else
        printWaitStatus(nullptr, ret); 
    sleep(5); 
    ret = power_on_usb();
    if (ret == -1)
        cout << "a child process could not be created, or"
                "its status could not be retrieved!" << endl;
    else
        printWaitStatus(nullptr, ret); 
    return EXIT_SUCCESS;
}
