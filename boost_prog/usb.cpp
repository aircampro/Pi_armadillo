//
// example of resetting usb link without using "system call"
//
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/wait.h>

int main(int argc, char *argv[]){
  int pid;
  int code;
  int status;
  pid_t result;
  int i = 0;

  while(i < 2){
    pid = fork();
    
    // fork
    if(pid == -1){
      fprintf(stderr, "Error\n\n");
    }
    char *argv[6];
    //char *argv[2];
    // sudo /home/pi/cams/SonyTEST32/uhubctl/uhubctl -l 1-1 -a 0
    argv[0] = "/home/pi/cams/SonyTEST32/uhubctl/uhubctl";
    argv[1] = "-l";
    argv[2] = "1-1";
    argv[3] = "-a";
    // first time disable then enable the usb power
    if ( i == 0 ) {
        argv[4] = "0";
    } else {
        argv[4] = "1";
    }
    argv[5] = NULL;

    //return(execve(argv[0], argv, environ));    
    // Processing Child Processes
    if(pid == 0){
      //execl("/home/pi/cams/SonyTEST32/uhubctl/uhubctl", "/home/pi/cams/SonyTEST32/uhubctl/uhubctl", NULL);
      execve(argv[0], argv, environ);    
    }else{
      result = wait(&status);
      
     if(result < 0){
        fprintf(stderr, "Wait Error.\n\n");
        exit(-1);
     }
      
     //Check the Exit status 
     if(WIFEXITED(status)){
        printf("Child process Termination");
        code = WEXITSTATUS(status);
        printf("the code %d\n", code);
      }else{
        printf("wait failuer");
        printf("exit code : %d\n", status);
      }
      
      i++;
    }    

  }
  printf("Parent process Termination\n");
  return 0;
}
