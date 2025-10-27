// linux code to control rayfin camera this can be used in zybo or raspi with linux os for example
// rayfin ref:- https://www.subcservices.com/services/api/1.13.27/ & https://manuals.plus/m/935d753ee92b196b38af2b8523b29b9d9b6d554ef65db97f1126228060ae7f86.pdf
//
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#define DLAY 100

int main()
{
 int sock;
 struct sockaddr_in addr;

 sock = socket(AF_INET, SOCK_DGRAM, 0);

 addr.sin_family = AF_INET;
 addr.sin_port = htons(8887);                                                 // connect to the udp port for rayfin camera system
 addr.sin_addr.s_addr = inet_addr("192.168.1.12");

 printf("Connecting to Camera and taking pictures\n");
 sendto(sock, "OpenPort:8887", 13, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY);
 sendto(sock, "StartLogging", 12, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY);
 sendto(sock, "EnableStrobe", 12, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY);
 sendto(sock, "LampOn", 6, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY);
 // Switch to auto exposure and take 10 pictures 1 
 // minute apart 
 sendto(sock, "AutoExposure", 12, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY);
 sendto(sock, "Repeat:TakePicture|WaitFor:60000|10", 35, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY);
 usleep(1000000);
 // Record video for 1 minute
 sendto(sock, "VideoName:StillsTest", 20, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY);
 sendto(sock, "StartRecording", 14, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY); 
 sendto(sock, "WaitFor:60000", 13, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY);
 sendto(sock, "StopRecording", 14, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY); 
 sendto(sock, "StopLogging", 11, 0, (struct sockaddr *)&addr, sizeof(addr));
 usleep(DLAY); 
 close(sock);

 return 0;
}