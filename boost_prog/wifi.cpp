/*
   This example shows how to reset the wifi using system calls 
*/
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

extern char **environ;

/*
   example function calls execve after forking to connect the wifi if it fails and needs re-connection
*/
int connect_wifi(void)
{
    char *argv[8];

    printf("This is child(pid=%d)\n", getpid());

    argv[0] = "/bin/nmcli";
    argv[1] = "device";
    argv[2] = "wifi";
    argv[3] = "con";
    argv[4] = "rededgeRX03-2136114-SC";
    argv[5] = "password";
    argv[6] = "micasense";
    argv[7] = NULL;

    //return(execve(argv[0], argv, environ));
			  
    if (execve(argv[0], argv, environ) < 0) {
       perror("execve");
       return(-1);
    }
	else {
	   return 1;
	}
}

int main(void)
{
    int child, status;
    if ((child = fork()) < 0) {
       perror("fork");
       return(1);
    }

    if (child == 0)
        connect_wifi();
    else 
	{
        if (wait(&status) < 0) {
            perror("wait");
            return(1);
        }
        printf("The child (pid=%d) existed with status(%d).\n", child, WEXITSTATUS(status));
    }
}
