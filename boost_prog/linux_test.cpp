      #include <stdio.h>    
      #include <unistd.h>
      extern char **environ;
    
      int main(void)
      {
              char *argv[3];
    
              argv[0] = "sudo";
              argv[1] = "/bin/ls";
              argv[2] = NULL;
    
              return(execve(argv[0], argv, environ));
      }

