/*
   example to demonstrate reset of usb power on -> wait -> power on
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstring>

using std::cout; using std::endl;
using std::string; using std::cin;

/*
   uses uhubctrl to power on the usb hub 
*/
int power_on_usb(void)
{
    return(system("sudo /home/pi/cams/SonyTEST32/uhubctl/uhubctl -l 1-1 -a 1"));
}

/*
   uses uhubctrl to power off the usb hub 
*/
int power_off_usb(void)
{
    return(system("sudo /home/pi/cams/SonyTEST32/uhubctl/uhubctl -l 1-1 -a 0"));
}

int printWaitStatus(const char *msg, int status)
{
    if (msg != nullptr)
        printf("%s", msg);

    if (WIFEXITED(status)) {
        printf("child exited, status=%d\n", WEXITSTATUS(status));
        return EXIT_SUCCESS;
    } else if (WIFSIGNALED(status)) {
        printf("child killed by signal %d (%s)", WTERMSIG(status), strsignal(WTERMSIG(status)));
#ifdef WCOREDUMP
        if (WCOREDUMP(status))
            printf(" (core dumped)");
#endif
        printf("\n");
        return EXIT_FAILURE;
    } else if (WIFSTOPPED(status)) {
        printf("child stopped by signal %d (%s)\n", WSTOPSIG(status), strsignal(WSTOPSIG(status)));
        return EXIT_FAILURE;
#ifdef WIFCONTINUED
    } else if (WIFCONTINUED(status)) {
        printf("child continued\n");
        return EXIT_FAILURE;
#endif
    } else {
        printf("status=%x\n", (unsigned int) status);
        return EXIT_FAILURE;
    }
}

int main(void)
{
    int ret = power_off_usb();
    int r_code = EXIT_FAILURE;
    if (ret == -1)
        cout << "a child process could not be created, or"
                "its status could not be retrieved!" << endl;
    else
        r_code = printWaitStatus(nullptr, ret);
    sleep(5);
    ret = power_on_usb();
    if (ret == -1)
        cout << "a child process could not be created, or"
                "its status could not be retrieved!" << endl;
    else
        r_code = printWaitStatus(nullptr, ret);
    return EXIT_SUCCESS;
}
