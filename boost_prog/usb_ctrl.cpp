      #include <stdio.h>    
      #include <unistd.h>
      #include <iostream>
      extern char **environ;
    
      int usb_off(void)
      {
              char *argv[6];
              //char *argv[2];
              // sudo /home/pi/cams/SonyTEST32/uhubctl/uhubctl -l 1-1 -a 0 
              argv[0] = "/home/pi/cams/SonyTEST32/uhubctl/uhubctl"; 
              argv[1] = "-l";
              argv[2] = "1-1";
              argv[3] = "-a";
              argv[4] = "0";
              argv[5] = NULL;
    
              return(execve(argv[0], argv, environ));
              //return(execve(argv[0], argv, NULL));
      }

      int usb_on(void)
      {
              char *argv[6];
              //char *argv[2];
              // sudo /home/pi/cams/SonyTEST32/uhubctl/uhubctl -l 1-1 -a 0 
              argv[0] = "/home/pi/cams/SonyTEST32/uhubctl/uhubctl"; 
              argv[1] = "-l";
              argv[2] = "1-1";
              argv[3] = "-a";
              argv[4] = "1";
              argv[5] = NULL;
    
              return(execve(argv[0], argv, environ));
              //return(execve(argv[0], argv, NULL));
      }

      int main(void) {
    
          usb_off();
          std::cout << "\033[32m USB OFF \033[0m" << "\n";
          sleep(2);
          usb_on(); 
          std::cout << "\033[33m USB ON \033[0m" << "\n";
     }
