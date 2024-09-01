#ifdef defined(_WIN32) 
/**************************************************************************************************************************************
 Academic sCala Robot C Language Sample [Cartesian Coordinate Conversion]

 The CP2110/4 HID USB・to-UART interface library is required to build and run this sample.
 Download the software and documentation from the SiLabs web page (below) and install it on your PC.
 http://jp.silabs.com/products/interface/Pages/CP2110EK.aspx

 Once installed on your PC, open the properties of the VisualStudio project that contains this source and set the following settings.
 1．Change "Configuration" to "All Configurations"
 2．Click "Configuration Properties" -> "VC++ Directory" and add the path to the include file to "Include Directory"
 In the default installation settingsC:\SiLabs\MCU\CP2110_4_SDK\Library\Windows It will be "\".
 
 3．Also, add the path to the library to the "Library directory"
 There is a folder "x86" and "x64" in the path of the include file set above, so it is suitable for the target platform of the projectSelect either.
 To check the platform settings, open the project properties and review the "Platform" item.
 Visual Studio is set to "Win32" by default, in this case using "x86".
 
 4．Click "OK"
 5．In the program execution folder, select "SLABHIDDevice.dll」「SLABHIDtoUART.Copy the file of "dll"
 Use the file in the folder selected in the "Library Directory".

libs are as follows:-
if sys.platform == 'win32':
    g_DLL = ctypes.windll.LoadLibrary("./CP210xManufacturing.dll")
elif sys.platform.startswith('linux'):
    g_DLL = ctypes.cdll.LoadLibrary("./libcp210xmanufacturing.so.1.0")
elif sys.platform == 'darwin':
    g_DLL = ctypes.cdll.LoadLibrary("libCP210xManufacturing.dylib")

 When you send a communication message, a local echo is always generated.
 When acquiring information from the robot, such as motor angle, if you send a message for acquiring information from the PC、
 It reads all local echoes like the ReadLocalEcho function, and then receives a reply message from the robot.

 31/08/24 ACP edited for dual windows linuz use
	
**********************************/
#define _CRT_SECURE_NO_WARNINGS

/*-------------- Header file ----------------*/
#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//CP2110/4 HID USB-to-UART Interface Library Include file
#include <SLABCP2110.h>
#include <SLABHIDtoUART.h>
#include <CP2114_Common.h>

//Link to the CP2110/4 HID USB-to-UART Interface Library
#pragma comment(lib, "SLABHIDtoUART.LIB")
#pragma comment(lib, "winmm.LIB")

#define M_PI (3.1415926)

#elif defined(__linux__) || defined(__APPLE__)

/**************************************************************************************************
	
	Combined Apple / Linux Version
	uses ibslabhidtouart.so for CP2110/4 on linux
         libSLABHIDtoUART.dylib  for CP2110/4 on apple darwin	
		 
    There are 32bit and 64bit libs available
	
	2016.08.28
	@cryborg21
    This code includes the work that is distributed in
    https://www.vstone.co.jp/products/scara_robot/download.html#03-3
    https://github.com/cryborg21/vsasr_linux
  
    31/08/24 ACP edited for ubuntu razpai use
   
**************************************************************************************************/

/*--------------	libraries for linux	----------------*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

//CP2110/4 HID USB-to-UART this is for linux
#include <silabs/Types.h>
#include <silabs/SLABCP2110.h>
#include <silabs/SLABHIDtoUART.h>
#include <silabs/CP2114_Common.h>

#endif

/*-------------- Macro ----------------*/
/*---- Communication settings ----*/
//Vender ID of the robot
#define VID (0x10C4)

//Product ID of the robot
#define PID (0xEA80)

/*---- Settings related to the dimensions of the main unit ----*/
// Axial distance of the first joint (mm)
#define AXISLEN_A (80.0)
// Axial distance of the second joint (mm)
#define AXISLEN_B (80.0)

// Stage length width (mm)
#define FIELD_H (210.0)
// Stage width (mm)
#define FIELD_W (230.0)
// X coordinate (mm) of the base of the arm (ID1 motor output shaft) with the center of the stage as the origin (0,0)
#define X_OFS (FIELD_W/2 - 53.0)

// Vertical axis distance conversion factor
#define HEIGHT_RATE (148.54)
// Movable range of vertical axis (mm)
#define HEIGHT_RANGE (35.0)
// Macro function to convert from motor angle to distance.r= Motor angle value (in 0.1 degrees)
#define RAD_TO_HEIGHT(r) ( ((double)r/(HEIGHT_RATE*10.0))*HEIGHT_RANGE)
// Macro function to convert from distance to motor angle.h= Distance (mm)
#define HEIGHT_TO_RAD(h) (short) (h/HEIGHT_RANGE*HEIGHT_RATE*10.0)

// Hand axis distance conversion factor
#define WIDTH_RATE (31.83)
// Movable range of hand shaft (mm)
#define WIDTH_RANGE (5.0*2.0)
// Claw hole position width (mm)
#define GROW_W (5.0*2.0)
// Macro function to convert from motor angle to width.r= Motor angle value (in 0.1 degrees), p= nail mounting position (0-3)
#define RAD_TO_WIDTH(r,p) ( ((double)-r/(WIDTH_RATE*10.0))*WIDTH_RANGE + GROW_W*(p+1))
// Macro function to convert from width to servo angle.w= nail width (mm), p= nail mounting position (0-3)
#define WIDTH_TO_RAD(w,p) (short) (-(w-GROW_W*(p+1))/WIDTH_RANGE*WIDTH_RATE*10.0)

// Motor movable range of arm (in 0.1 degree units)
#define ARM_RAD_RANGE (1350)
// Movable range of motor of hand opening and closing shaft (in 0.1 degree units)
#define HAND_WIDTH_RANGE (350)

/*---- Coordinate system settings ----*/
// Body angle
#define BASE_ANGLE (180.0)
// Body position (mm)
#define BASE_OFFSET_X (+0.0)
#define BASE_OFFSET_Y (+0.0)

// The screw hole number of the nail of the current hand shaft (0~3 = ①~④)
#define CROW_POS (1)

/*---- Other necessary settings ----*/

// Transition time for moving the motor to the target position (msec)
#define MOVE_TIME (1000)

// 1 side length of obstacle (cube) (in mm)
#define CUBE_SZ (20.0)
// Lifting and lowering width of the arm up and down axis in program (mm)
#define ZMIN (-30.0)
#define ZMAX (20.0)
// Opening and closing width of hand shaft during program (mm)
#define GRIP_CLOSE (15.0)
#define GRIP_OPEN (30.0)

/*-------------- functions ---------------*/
int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num);
int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam);
int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num);
int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,unsigned int data_len);
int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num);
void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num);
int SetTXOpenDrain(HID_UART_DEVICE dev );
#if defined(__linux__) 
void sigintFunc(int sig);
#endif

/*--------------		main			 ---------------*/

/*-------------- Processing ----------------*/
void main()
{
 DWORD numDevice=0;
 HID_UART_DEVICE dev=0; // Communication handle

#if defined(__linux__)
 signal(SIGINT, sigintFunc);
#endif

 printf("SCARA-Robot sample 'Pick up & carry Cubes'.\n");

 // Get the number of robots currently connected to the PC
 HidUart_GetNumDevices(&numDevice,VID,PID);
 printf("%d device(s) found.\n",numDevice);

 //Exit the program if 1 unit is not connected
 if(numDevice==0) return;

 // Connect to 1st robot
 if(HidUart_Open(&dev,0,VID,PID)!=HID_UART_SUCCESS){
 //Exit if connection is not done correctly
 printf("cannot open device\n");
 return ;
 }
 else{
 UINT uPeriod=10; //Sleep関数の分解能（default=16msec -> 10msec）

 //TX changed to Open-Drain
 SetTXOpenDrain(dev);
 
 // Set the gain to ON
 printf("Motor torque ON\n");
 RSTorqueOnOff(dev,1,1,5);

 // Time resolution setting
 timeBeginPeriod(uPeriod);

 // Main loop
 {

 struct operation{ // structure corresponding to 1 instruction
 double x,y,z,yaw,w; // Cartesian coordinates of the target and hand axis angle・hand opening and closing width
 };

 struct operation *p=NULL,prep;
 int op=0,i;
 short SPOs[5]; // array variable for specifying the target position of the motor
 int demoType;

 printf("Input demo type(0 - 4):");
 scanf("%d",&demoType);

 switch(demoType){
 case 0: // Distribute the blocks piled up at (0,0) in three directions
 case 1: // Pile up blocks distributed in three directions to (0,0)
 op=8*3;
 p = (struct operation*) calloc(sizeof(struct operation),op);

 if(demoType==0){
 for(i=0;i<op;i++) p[i].yaw=90;
 }

 for(i=0;i<3;i++){
 int t=i*8;
 int tx=0,ty=0;

 switch(i){
 case 0:
 tx=0 ; ty= -60;
 break;
 case 1:
 tx=-60; ty= 0;
 break;
 case 2:
 tx=0 ; ty= 60;
 break;
 }

 if(demoType==0){
 p[t+0].x = 0; p[t+0].y = 0; p[t+0].z = ZMAX;
 p[t+0].w=GRIP_OPEN;

 p[t+1].x = 0; p[t+1].y = 0; p[t+1].z = ZMIN+CUBE_SZ*(2-i);
 p[t+1].w=GRIP_OPEN;

 p[t+2].x = 0; p[t+2].y = 0; p[t+2].z = ZMIN+CUBE_SZ*(2-i);
 p[t+2].w=GRIP_CLOSE;

 p[t+3].x = 0; p[t+3].y = 0; p[t+3].z = ZMAX;
 p[t+3].w=GRIP_CLOSE;

 p[t+4].x = tx; p[t+4].y = ty; p[t+4].z = ZMAX;
 p[t+4].w=GRIP_CLOSE;
 p[t+5].x = tx; p[t+5].y = ty; p[t+5].z = ZMIN;
 p[t+5].w=GRIP_CLOSE;

 p[t+6].x = tx; p[t+6].y = ty; p[t+6].z = ZMIN;
 p[t+6].w=GRIP_OPEN;
 p[t+7].x = tx; p[t+7].y = ty; p[t+7].z = ZMAX;
 p[t+7].w=GRIP_OPEN;
 }
 else{
 p[t+0].x = tx; p[t+0].y = ty; p[t+0].z = ZMAX;
 p[t+0].w=GRIP_OPEN;
 p[t+1].x = tx; p[t+1].y = ty; p[t+1].z = ZMIN;
 p[t+1].w=GRIP_OPEN;

 p[t+2].x = tx; p[t+2].y = ty; p[t+2].z = ZMIN;
 p[t+2].w=GRIP_CLOSE;
 p[t+3].x = tx; p[t+3].y = ty; p[t+3].z = ZMAX;
 p[t+3].w=GRIP_CLOSE;

 p[t+4].x = 0; p[t+4].y = 0; p[t+4].z = ZMAX;
 p[t+4].w=GRIP_CLOSE;

 p[t+5].x = 0; p[t+5].y = 0; p[t+5].z = ZMIN+CUBE_SZ*(i);
 p[t+5].w=GRIP_CLOSE;

 p[t+6].x = 0; p[t+6].y = 0; p[t+6].z = ZMIN+CUBE_SZ*(i);
 p[t+6].w=GRIP_OPEN;

 p[t+7].x = 0; p[t+7].y = 0; p[t+7].z = ZMAX;
 p[t+7].w=GRIP_OPEN;
 }
 }
 break;
 case 2: // (0,0) stacked blocks horizontally
 op=8*3;
 p = (struct operation*) calloc(sizeof(struct operation),op);

 for(i=0;i<3;i++){
 int t=i*8;
 p[t+0].x = 0; p[t+0].y = 0; p[t+0].z = ZMAX;
 p[t+0].w=GRIP_OPEN;
 p[t+1].x = 0; p[t+1].y = 0; p[t+1].z = ZMIN+CUBE_SZ*(2-i);
 p[t+1].w=GRIP_OPEN;

 p[t+2].x = 0; p[t+2].y = 0; p[t+2].z = ZMIN+CUBE_SZ*(2-i);
 p[t+2].w=GRIP_CLOSE;
 p[t+3].x = 0; p[t+3].y = 0; p[t+3].z = ZMAX;
 p[t+3].w=GRIP_CLOSE;

 p[t+4].x = -60; p[t+4].y = 0; p[t+4].z = ZMAX;
 p[t+4].w=GRIP_CLOSE;
 p[t+5].x = -60; p[t+5].y = 0; p[t+5].z = ZMIN+CUBE_SZ*(i);
 p[t+5].w=GRIP_CLOSE;

 p[t+6].x = -60; p[t+6].y = 0; p[t+6].z = ZMIN+CUBE_SZ*(i);
 p[t+6].w=GRIP_OPEN;
 p[t+7].x = -60; p[t+7].y = 0; p[t+7].z = ZMAX;
 p[t+7].w=GRIP_OPEN;
 }
 break;

 default: // Stack horizontally stacked blocks back to (0,0)
 op=8*3;
 p = (struct operation*) calloc(sizeof(struct operation),op);

 for(i=0;i<op;i++) p[i].yaw=90;

 for(i=0;i<3;i++){
 int t=i*8;
 p[t+0].x = -60; p[t+0].y = 0; p[t+0].z = ZMAX;
 p[t+0].w=GRIP_OPEN;
 p[t+1].x = -60; p[t+1].y = 0; p[t+1].z = ZMIN+CUBE_SZ*(2-i);
 p[t+1].w=GRIP_OPEN;
 p[t+2].x = -60; p[t+2].y = 0; p[t+2].z = ZMIN+CUBE_SZ*(2-i);
 p[t+2].w=GRIP_CLOSE;
 p[t+3].x = -60; p[t+3].y = 0; p[t+3].z = ZMAX;
 p[t+3].w=GRIP_CLOSE;

 p[t+4].x = 0; p[t+4].y = 0; p[t+4].z = ZMAX;
 p[t+4].w=GRIP_CLOSE;
 p[t+5].x = 0; p[t+5].y = 0; p[t+5].z = ZMIN+CUBE_SZ*(i);
 p[t+5].w=GRIP_CLOSE;

 p[t+6].x = 0; p[t+6].y = 0; p[t+6].z = ZMIN+CUBE_SZ*(i);
 p[t+6].w=GRIP_OPEN;
 p[t+7].x = 0; p[t+7].y = 0; p[t+7].z = ZMAX;
 p[t+7].w=GRIP_OPEN;
 }
 }

 // Move all joints to the state of the first command
 pos_to_rad(p[0].x,p[0].y,p[0].z,p[0].yaw,p[0].w,sPos,-1,5);

 RSMove(dev,sPos,(unsigned short)MOVE_TIME/10,1,5);
 Sleep(MOVE_TIME);

 memcpy(&prep,&p[0],sizeof(struct operation));

 Sleep(500);

 for(int i=1;i<op;i++){

 // Move to target pose
 printf("(x=%.2f,y=%.2f,z=%.2f,yaw=%.2f,width=%.2f)",p[i].x,p[i].y,p[i].z,p[i].yaw,p[i].w);
 pos_to_rad(p[i].x,p[i].y,p[i].z,p[i].yaw,p[i].w,sPos,-1,5);

 RSMove(dev,sPos,MOVE_TIME/10,1,5);
 Sleep(MOVE_TIME);

 // Exit if the interrupt flag is standing
 if( (GetKeyState(VK_ESCAPE) & 0x80) || (GetKeyState('Q') & 0x80) ){
 printf("Aboat\n");
 break;
 }

 memcpy(&prep,&p[i],sizeof(struct operation));

 printf("\n");
 }

 if(p) free(p);
 }

 // Turn the gain OFF
 RSTorqueOnOff(dev,0,1,5);

 // Restore Time resolution
 timeEndPeriod(uPeriod);
 }

 // Disconnect communication with the robot
 printf("close device\n");
 HidUart_Close(dev);

}

#if defined(__linux__)
/* ctrl + c clean-up handler */
void sigintFunc(int sig)
{
	//torque OFF
	RSTorqueOnOff(dev,0,1,5);
	printf("Gain OFF.\n");

	//close CP2110/4 uart connection
	printf("close device\n");
	HidUart_Close(dev);

	exit(0);
}
#endif

/* common libraries */

/************************************************

 void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num)
 
 Summary: A function that converts a given cartesian coordinate (X/Y/Z) and the angle and opening/closing width of the hand axis to the target position of the motor.

 Arguments：
 double x,y,z … position vector
 double yaw angle of the original hand rotation axis
 double w  Width of the original hand opening and closing axis
 short SPOs Pointer to array variable to assign motor angle after calculation
 int sign The direction of the arm bending.Positive values bend clockwise, negative values bend counterclockwise
 int num Number of motors to use

 Return value：
 None

*************************************************/
void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num)
{

 double a,b,tx,ty,lx,ly; // variables to find the cosine theorem
 double thete1,thete2; //variable to find the angle of ID1,ID2
 double s; // The variable to use for the direction (sign) of the arm bending.Assign +1.0 or -1.0 depending on the sign of int sign

 //Determine the bending direction of the arm from the sign of the argument
 if(sign<0) s=-1.0;
 else s=1.0;

 //Reflect the position and angle settings of the body (reverse the conversion formula and convert both orientation and position to values relative to the position of 0)
 lx= x-BASE_OFFSET_X;
 ly= y-BASE_OFFSET_Y;
 x = (lx*cos(-BASE_ANGLE/180.0*M_PI)-ly*sin(-BASE_ANGLE/180.0*M_PI));
 y = (lx*sin(-BASE_ANGLE/180.0*M_PI)+ly*cos(-BASE_ANGLE/180.0*M_PI));

 ty = y;
 tx = x+X_OFS;

 //If the target value is farther than the link length, extend the arm by adjusting the root axis to the angle to the target value
 if(hypot(tx,ty)>=AXISLEN_A+AXISLEN_B){

 sPos[0] = (short) (atan2(ty,tx)/M_PI*1800.f);
 sPos[1] = 0;

 } else {

 //Compute the cosine theorem
 a = acos( (-(tx*tx+ty*ty) + AXISLEN_A*AXISLEN_A + AXISLEN_B*AXISLEN_B) / (2 * AXISLEN_A * AXISLEN_B));
 b = s*acos( (- AXISLEN_A*AXISLEN_A + AXISLEN_B*AXISLEN_B + (tx*tx+ty*ty)) / (2 * AXISLEN_A * sqrt(tx*tx+ty*ty)));

 // Select the angle calculation method by the sign of the specified X/Y coordinates
 if(tx<0){
 if(ty>=0) thete1 = M_PI + atan(ty/tx) +b;
 else thete1 = atan(ty/tx) + b - M_PI;
 }
 else thete1 = atan(ty/tx)+b;
 thete2 = -(M_PI-a)*s;

 // Convert angle to 360 degree method
 if(thete1!=0.0) thete1 = thete1 / M_PI * 180.0;
 if(thete2!=0.0) thete2 = thete2 / M_PI * 180.0;

 //Convert the converted angle to 0.1 degree units and change the sign according to the motor output direction (ID1)
 sPos[0] = (short) (thete1*10.0);
 sPos[1] = (short) (thete2*10.0);

 }
 // Limit the range of movement of 2 axes of the arm
 if(sPos[0]<-ARM_RAD_RANGE) sPos[0]=-ARM_RAD_RANGE;
 else if(sPos[0]>ARM_RAD_RANGE) sPos[0]=ARM_RAD_RANGE;

 if(sPos[1]<-ARM_RAD_RANGE) sPos[1]=-ARM_RAD_RANGE;
 else if(sPos[1]>ARM_RAD_RANGE) sPos[1]=ARM_RAD_RANGE;

 //Z Coordinate conversion
 sPos[2] = HEIGHT_TO_RAD(z);

 // Hand 2-axis conversion
 if(num>3){
 // Grip width
 sPos[4] = WIDTH_TO_RAD(w,CROW_POS);

 if(sPos[4]<-HAND_WIDTH_RANGE) sPos[4]=-HAND_WIDTH_RANGE;
 else if(sPos[4]>HAND_WIDTH_RANGE) sPos[4]=HAND_WIDTH_RANGE;

 sPos[3] = -sPos[0]-sPos[1];
 sPos[3] += (short) (yaw*10.0);
 }

}

/************************************************

 int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num)
 
 Summary: A function to switch the torque of the motor.Multiple motors with consecutive IDs can be set simultaneously.

 Arguments：
 HID_UART_DEVICE dev dev Communication handle
 short sMode指定 Specifies the ON/OFF of the gain.0=off, 1=ON
 BYTE idID ID of the motor to start switching (ID of the first motor when switching multiple motors)
 int num数 Number of motors to switch

 Return value：
 TRUE if the operation is successful; FALSE otherwise

*************************************************/
int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num)
{
 unsigned char sendbuf[256],*bufp;  // send buffer relation
 unsigned char sum;                 // for checksum calculation
 int ret;                           // for return value recording
 DWORD data_len=0,len=0;            // Get packet length and write size
 unsigned char i;

 // Clear Send buffer
 memset( sendbuf, 0x00, sizeof( sendbuf ));

 //Packet creation
 //1．Create Header-intersection
 sendbuf[0] = (unsigned char)0xFA;   // ヘッダー1
 sendbuf[1] = (unsigned char)0xAF;   // ヘッダー2
 sendbuf[2] = (unsigned char)0x00;   // サーボID(常に0)
 sendbuf[3] = (unsigned char)0x00;   // flag (always 0)
 sendbuf[4] = (unsigned char)0x24;   // アドレス(トルクON/OFF 0x24=36)
 sendbuf[5] = (unsigned char)0x01+1; // 長さ(1byte)
 sendbuf[6] = (unsigned char)num;    // モータの個数

 //Record packet length of intersection
 data_len = 7; 

 //2．Packet creation of servo individual parts
 bufp = &sendbuf[7]; // Assign the start address of the individual message portion of the send buffer

 //Add individual packets for the number of motors to be rewritten
 for(i=0;i<num;i++){
 *bufp++ = id+i; //ID of the motor
 data_len++; // Add 1byte packet length

 *bufp++ = (unsigned char)(sMode&0x00FF); //ON・OFFフラグ
 data_len++; // Add 1byte packet length
 }

 //3．Checksum Calculation
 /The /checksum is the value obtained by XOR the 3rd byte (servo ID)- end of the transmit buffer by 1byte.
 sum = sendbuf[2];
 for( i = 3; i < data_len; i++ ){
 sum = (unsigned char)(sum ^ sendbuf[i]);
 }
 sendbuf[data_len] = sum; // Assign the found checksum to the end of the send buffer
 data_len++; // Add 1byte packet length

 //4．Sending a Message
 ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );

 if(ret!=HID_UART_SUCCESS) return FALSE;

 //5．Read Local Echo
 return ReadLocalEcho(dev,sendbuf,data_len);
}

/************************************************

 int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam)
 
 Summary: Get the current position of the motor.The acquisition of the current position supports only 1 motor at the same time.

 Arguments：
 HID_UART_DEVICE dev dev Communication handle
 BYTE idID the ID of the motor to get the current position
 short *getParamる A pointer to a variable that stores the acquired current position

 Return value：
 TRUE if the operation is successful; FALSE otherwise

*************************************************/
int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam)
{
 unsigned char sendbuf[32];
 unsigned char readbuf[128];
 unsigned char sum;
 DWORD i;
 int ret;
 unsigned long len, readlen;
 short angle;

 // Clear buffer
 memset( sendbuf, 0x00, sizeof( sendbuf ));

 // Packet creation
 sendbuf[0] = (unsigned char)0xFA; // ヘッダー1
 sendbuf[1] = (unsigned char)0xAF; // ヘッダー2
 sendbuf[2] = (unsigned char)id; // サーボID
 sendbuf[3] = (unsigned char)0x0f; // flag (0x0f= get specified byte from specified address)
 sendbuf[4] = (unsigned char)0x2a; // the address of the fetched data (current position=0x2a)
 sendbuf[5] = (unsigned char)0x02; // length of data to get (current position =2byte)
 sendbuf[6] = (unsigned char)0x00; // 個数(0)

 // Checksum calculation
 sum = sendbuf[2];
 for( i = 3; i < 7; i++ ){
 sum = (unsigned char)(sum ^ sendbuf[i]);
 }
 sendbuf[7] = sum; // checksum

 // Send packet
 ret = HidUart_Write(dev,sendbuf, 8, &len);

 //Read local echo
 if(!ReadLocalEcho(dev,sendbuf,len)) return FALSE;

 // If the packet length that could be sent is less than the data size, an error
 if( len < 8 ) return FALSE;

 // Read receive buffer
 memset( readbuf, 0x00, sizeof( readbuf ));

 //Calculate packet length of receive buffer
 // Header(2byte) + ID(1byte) + Flags(1byte) + Address(1byte) + Length(1byte) + Count(1byte) + Dada(2byte) + Sum(1byte)
 readlen = (2) + (1) + (1) + (1) + (1) + (1) + (2) + (1);
 len = 0;

 //Receiving the buffer
 HidUart_Read(dev,readbuf, readlen, &len);

 //If the packet length of the receive buffer is different from the calculated length (readlen), an error occurs
 if( len < readlen) return FALSE;

 // Checksum confirmation of received data
 sum = readbuf[2];
 for( i = 3; i < readlen; i++ ){
 sum = sum ^ readbuf[i];
 }

 //If the checksum is different, error
 if( sum ) return FALSE;

 //Retrieve the read current position data from the receive buffer
 angle = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);
 if(getParam) *getParam = angle;

 return TRUE;
}

/************************************************

 int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num)
 
 Summary: Move the motor to a specified angle.It is possible to move multiple motors with consecutive IDs at once.

 Arguments：
 HID_UART_DEVICE dev dev Communication handle
 short *sPossイ Pointer to an array variable recording the target position.The target value of the start ID is used in order from the beginning of the buffer
 unsigned short sTime遷移 transition time to target position (in 10 milliseconds)
 BYTE idID The ID of the motor to be moved (if multiple motors are moved, the ID of the first motor)
 int numモ Number of motors to move

 Return value：
 TRUE if the operation is successful; FALSE otherwise

*************************************************/
int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num)
{
 unsigned char sendbuf[256],*bufp;
 unsigned char sum;
 unsigned char i;
 int ret;
 unsigned long len,data_len;

 // Clear buffer
 memset( sendbuf, 0x00, sizeof( sendbuf ));

 // Packet creation
 sendbuf[0] = (unsigned char)0xFA; // ヘッダー1
 sendbuf[1] = (unsigned char)0xAF; // ヘッダー2
 sendbuf[2] = (unsigned char)0; // ID(0)
 sendbuf[3] = (unsigned char)0x00; // フラグ(0x00)
 sendbuf[4] = (unsigned char)0x1E; // アドレス(0x1E=30)
 sendbuf[5] = (unsigned char)0x04+1; // 長さ(4byte)
 sendbuf[6] = (unsigned char)num; // モータの個数

 //Record packet length of intersection
 data_len = 7;

 //Individual data creation
 bufp = &sendbuf[7];
 for(i=0;i<num;i++){
 *bufp++ = id+i; //motor ID
 data_len++; // Add 1byte packet length

 //Write target position to buffer (2byte)
 *bufp++ = (unsigned char)(sPoss[i]&0x00FF);
 *bufp++ = (unsigned char)((sPoss[i]&0xFF00)>>8);
 data_len+=2; // Add 2byte packet length

 //Write transition time to buffer (2byte)
 *bufp++ = (unsigned char)(sTime&0x00FF);
 *bufp++ = (unsigned char)((sTime&0xFF00)>>8);
 data_len+=2; // Add 2byte packet length
 }

 // Checksum calculation
 sum = sendbuf[2];
 for( i = 3; i < data_len; i++ ){
 sum = (unsigned char)(sum ^ sendbuf[i]);
 }
 sendbuf[data_len] = sum; // Add checksum to send buffer
 data_len++; // Add 1byte packet length

 // Send packet
 ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
 if(ret!=HID_UART_SUCCESS) return FALSE;

 //Read Local Echo
 return ReadLocalEcho(dev,sendbuf,data_len);

}

/************************************************

 int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,DWORD data_len)
 
 Overview：
 A function that reads the local echo of an outgoing message.
 The message is received by the length of the data_len argument, and the contents of the *sendbuf argument are the same or compared

 Arguments：
 HID_UART_DEVICE dev dev Communication handle
 unsigned char *sendbufイ Pointer to sent message content
 DWORD data_len 送信 Size of outgoing message

 Return value：
 TRUE if the same content as the outgoing message is received; FALSE otherwise

*************************************************/
int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,DWORD data_len)
{

 unsigned char readbuf[1024];
 DWORD len=0;
 memset(readbuf,0,sizeof(readbuf));

 /Receive a message of the size of /data_len
 HidUart_Read( dev, (BYTE*) readbuf, data_len, &len );

 //Error if the size of the received message is different
 if(data_len!=len) return FALSE;

 //Compare incoming and outgoing messages
 for(DWORD i=0;i<len;i++){

 //If the received and sent messages are different, an error
 if(readbuf[i]!=sendbuf[i]) return FALSE;
 }
 return TRUE;
}

/************************************************

 int int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num)
 
 Overview：
 Rewrite the parameters of the motor.Multiple motors in succession ・ can be written to multiple addresses
 Specify address, size, number of motors・ and actual data

 Arguments：
 HID_UART_DEVICE dev dev Communication handle
 BYTE address BYTE Address to write data (See RS304MD for details)
 BYTE size ... The size of the data to be written per 1 motor (in bytes)
 BYTE idID The ID of the motor to be moved (if multiple motors are moved, the ID of the first motor)
 BYTE *dataき Pointer to array variable of write data contents
 int numモ Number of motors to move

 Return value：
 TRUE if the operation is successful; FALSE otherwise

*************************************************/
int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num)
{
 unsigned char sendbuf[256],*bufp;
 unsigned char sum;
 unsigned char i,j;
 int ret;
 unsigned long len,data_len;

 // Clear buffer
 memset( sendbuf, 0x00, sizeof( sendbuf ));

 // Packet creation
 sendbuf[0] = (unsigned char)0xFA; // ヘッダー1
 sendbuf[1] = (unsigned char)0xAF; // ヘッダー2
 sendbuf[2] = (unsigned char)0; // ID(0)
 sendbuf[3] = (unsigned char)0x00; // フラグ(0x00)
 sendbuf[4] = (unsigned char)address; // アドレス(0x1E=30)
 sendbuf[5] = (unsigned char)size+1; // 長さ(4byte)
 sendbuf[6] = (unsigned char)num; // モータの個数

 //Record packet length of intersection
 data_len = 7;

 //Individual data creation
 bufp = &sendbuf[7];
 for(i=0;i<num;i++){
 *bufp++ = id+i; //motor ID
 data_len++; // Add 1byte packet length

 for(j=0;j<size;j++){
 *bufp++ = (unsigned char)data[j]; // Add data part to packet by 1byte
 data_len++; // Add 2byte packet length
 }
 }

 // Checksum calculation
 sum = sendbuf[2];
 for( i = 3; i < data_len; i++ ){
 sum = (unsigned char)(sum ^ sendbuf[i]);
 }
 sendbuf[data_len] = sum; // Add checksum to send buffer
 data_len++;              // Add 1byte packet length

 // Send packet
 ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
 if(ret!=HID_UART_SUCCESS) return FALSE;

 //Read Local Echo
 return ReadLocalEcho(dev,sendbuf,data_len);

}
/************************************************

 int SetTXOpenDrain(HID_UART_DEVICE dev )
 
 Overview：
 Change TX to Open-Drain for USB-to-UART (CP2110) port settings.
 This setting is always necessary when communicating with the robot body (it is recorded in the robot body when changing the setting).

 Arguments：
 HID_UART_DEVICE dev dev Communication handle

 Return value：
 Each number defined in HID_UART_STATUS.Success if HID_UART_SUCCESS, otherwise failure

*************************************************/
int SetTXOpenDrain(HID_UART_DEVICE dev )
{
 /TX must be set to Open-Drain in the /GPIO configuration
 BYTE pinConfig[13];
 BOOL useSuspendValues;
 WORD suspendValue;
 WORD suspendMode;
 BYTE rs485Level;
 BYTE clkDiv;

 //Get the current Pin Config
 HidUart_GetPinConfig(dev, pinConfig, &useSuspendValues, &suspendValue, &suspendMode, &rs485Level, &clkDiv);

 //TX changed to TX-Open-Drain(0x01)
 pinConfig[10] = 0x01;

 // Update Pin Config
 return HidUart_SetPinConfig(dev, pinConfig, useSuspendValues, suspendValue, suspendMode,rs485Level, clkDiv);

}
