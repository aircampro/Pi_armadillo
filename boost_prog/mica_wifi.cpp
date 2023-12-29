      #include <stdio.h>    
      #include <unistd.h>
      extern char **environ;
    
      int main(void)
      {
              char *argv[8];
              //char *argv[2];
    
              //argv[0] = "sudo";
              argv[0] = "/bin/nmcli";
              //argv[1] = NULL;
              argv[1] = "device";
              argv[2] = "wifi";
              argv[3] = "con";
              argv[4] = "rededgeRX03-2136114-SC";
              argv[5] = "password";
              argv[6] = "micasense";
              argv[7] = NULL;
    
              return(execve(argv[0], argv, environ));
              //return(execve(argv[0], argv, NULL));
      }

