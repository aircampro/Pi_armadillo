#include <iostream>
#include <string>
#include<errno.h>
#include<stdio.h>
#include<sys/sysinfo.h>
#include<linux/unistd.h>
#include<linux/kernel.h>

typedef union
{
   std::uint32_t i;
   char xchar[4];
} union_t;
 
long get_uptime();

long get_uptime()
{
struct sysinfo s_info;
int error=sysinfo(&s_info);
if(error!=0)
{
//printf("code error=%d\ n",error);
std::cout << error << "\n";

}
return s_info.uptime;
}
int main(int argc,char *argv[])
{
double time;
long long time1;
time=get_uptime();
time1=get_uptime();
//printf("%8.0f\n",time);
std::cout << time << "as int " << (std::uint64_t) time << " " << time1 << "\n";
int param_value = 112257;
union_t u;
u.xchar[0] = param_value & 0xff;
u.xchar[1] = (char)(param_value >> 8) & 0xFF;
u.xchar[2] = (char)(param_value >> 16) & 0xFF;
u.xchar[3] = (char)(param_value >> 24) & 0xFF;
std::cout << "union " << u.i << std::endl;
return 0;
}
