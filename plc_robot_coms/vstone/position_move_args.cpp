/************************************************************
    This code controls robot from Vstone called VS-ASR 
	Linux
	2016.08.28
	@cryborg21
	2025.10.07
	@acp
  This code includes the work that is distributed in
  https://www.vstone.co.jp/products/scara_robot/download.html#03-3
  https://cryborg.hatenablog.com/entry/2016/09/04/153224
************************************************************/

/*--------------	library includes	----------------*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

//CP2110/4 HID USB-to-UART 
#include <silabs/SLABCP2110.h>
#include <silabs/SLABHIDtoUART.h>
#include <silabs/CP2114_Common.h>

/*-------------- for command line parsing of arguments ----------------*/
#include <iostream>
#include <random>
#include <tclap/CmdLine.h>

/*--------------	defines		----------------*/
//Vender ID
#define	VID	(0x10C4)
#define	PID	(0xEA80)
#define	AXISLEN_A (80.0)
#define	AXISLEN_B (80.0)
#define	FIELD_H	(210.0)
#define	FIELD_W	(230.0)
#define	X_OFS (FIELD_W/2 - 53.0)
#define	HEIGHT_RATE	(148.54)
#define	HEIGHT_RANGE (35.0)
#define	RAD_TO_HEIGHT(r) ( ((double)r/(HEIGHT_RATE*10.0))*HEIGHT_RANGE)
#define	HEIGHT_TO_RAD(h) (short) (h/HEIGHT_RANGE*HEIGHT_RATE*10.0)
#define	WIDTH_RATE	(31.83)
#define	WIDTH_RANGE	(5.0*2.0)
#define	GROW_W	(5.0*2.0)
#define	RAD_TO_WIDTH(r,p) ( ((double)-r/(WIDTH_RATE*10.0))*WIDTH_RANGE + GROW_W*(p+1))
#define	WIDTH_TO_RAD(w,p) (short) (-(w-GROW_W*(p+1))/WIDTH_RANGE*WIDTH_RATE*10.0)
#define	ARM_RAD_RANGE (1350)
#define	HAND_WIDTH_RANGE (350)
#define	CROW_POS	(1)
#define	SVPOS_TO_RAD(p)	(((double)(p)/1800.0)*M_PI)

/*-------------- function definitions ---------------*/
int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num);
int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam);
void rad_to_pos(double *x, double *y, double *z, double *yaw, double *w, short *sPos, int num, double base_angle, double base_offset_x, double base_offset_y);
void getPosition(double *x, double *y, double *z, double *yaw, double *width, double base_angle, double base_offset_x, double base_offset_y);
int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num);
int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,unsigned int data_len);
int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num);
void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num, double base_angle, double base_offset_x, double base_offset_y );
int SetTXOpenDrain(HID_UART_DEVICE dev );
void sigintFunc(int sig);

double	base_angle=180.0;
double	base_offset_x=0.0;
double	base_offset_y=0.0;
int servoNum = 0;		
double tx=0.0,ty=0.0,tz=0.0,tyaw=0.0,tw=0.0;
unsigned int numDevice=0;                
HID_UART_DEVICE dev=0;	                 
int move_time = 1000;                     // ms
/*--------------		main program			 ---------------*/
int main(int argc, char *argv[])
{

	printf("SCARA-Robot sample 'position to motor-axis-param'.\n");
	// ctrl+c signal handler
	signal(SIGINT, sigintFunc);

	try {  
	    TCLAP::CmdLine cmd("position_move_args", ' ', "0.1");
	    TCLAP::ValueArg<int> snArg("sn", "servo_num", "number of servos", false, 5, "int");
	    TCLAP::ValueArg<int> mtArg("mt", "mov_time", "move time", false, 1000, "int");
	    TCLAP::ValueArg<double> baArg("ba", "base_angle", "base angle", false, 180, "double");
	    TCLAP::ValueArg<double> bxArg("bx", "base_angle_ox", "base angle offset x", false, 0, "double");
	    TCLAP::ValueArg<double> byArg("by", "base_angle_oy", "base angle offset y", false, 0, "double");
	    TCLAP::ValueArg<double> txArg("tx", "x", "move positon x", false, 1.0, "double");
	    TCLAP::ValueArg<double> tyArg("ty", "y", "move positon y", false, 1.0, "double");
	    TCLAP::ValueArg<double> tzArg("tz", "z", "move positon z", false, 1.0, "double");
	    TCLAP::ValueArg<double> tyawArg("y", "yaw", "move positon yaw", false, 1.0, "double");
	    TCLAP::ValueArg<double> twArg("w", "width", "move positon width", false, 1.0, "double");

	    cmd.add( snArg );
	    cmd.add( mtArg );
	    cmd.add( baArg );
	    cmd.add( bxArg );
	    cmd.add( byArg );
	    cmd.add( txArg );
	    cmd.add( tyArg );
	    cmd.add( tzArg );
	    cmd.add( tyawArg );
	    cmd.add( twArg );

	    cmd.parse( argc, argv );

	    servoNum = snArg.getValue();
	    move_time = mtArg.getValue();
	    base_angle = baArg.getValue();
	    base_offset_x = bxArg.getValue();
	    base_offset_y = byArg.getValue();
	    tx = txArg.getValue();
	    ty = tyArg.getValue();
	    tz = tzArg.getValue();
	    tyaw = tyawArg.getValue();
	    tw = twArg.getValue();	
	}
    catch (TCLAP::ArgException &e) 
	{ 
        if(FLAG_DEBUG) {
            cerr << "error: " << e.error() << " for arg " << e.argId() << endl; 
        }
    }
	
	HidUart_GetNumDevices(&numDevice,VID,PID);
	printf("%d device(s) found.\n",numDevice);

	if(numDevice==0) return 0;

	if(HidUart_Open(&dev,0,VID,PID)!=HID_UART_SUCCESS){
		printf("cannot open device\n");
		return 1;
	}
	else{

		SetTXOpenDrain(dev);

	    if(servoNum!=3 && servoNum!=5) {
             printf("Does not correspond to %d motors.\n", servoNum);
			 return -1;
        }
		printf("%d-axis type\n",servoNum);

		printf("Gain ON.\n");
		RSTorqueOnOff(dev, 1, 1, servoNum);

	    short sPos[2];
		int sign=-1;

		pos_to_rad(tx, ty, tz, tyaw, tw, sPos, sign, servoNum, base_angle, base_offset_x, base_offset_y);

		if(servoNum>3) printf("  move to (%lf,%lf,%lf,%lf,%lf)\n",tx,ty,tz,tyaw,tw);
		else printf("  move to (%lf,%lf,%lf)\n",tx,ty,tz);

		for(int i=0;i<servoNum;i++){
			printf("ID%d=%+d ",i,sPos[i]);
		}
		printf("\n");
		RSMove(dev, sPos, move_time/10, 1, servoNum);
		sleep(2*(move_time/1000)); 
		RSTorqueOnOff(dev, 0, 1, servoNum);
	}

	double x = 0;
	double y = 0;
	double z = 0;
	double yaw = 0;
	double width = 0;
	getPosition(&x, &y, &z, &yaw, &width, base_angle, base_offset_x, base_offset_y);
	printf("X:%+6.2fmm, Y:%+6.2fmm, Z:%+6.2fmm", x, y, z);
	if (servoNum == 5)
	    printf(", Yaw:%+6.2fdeg, Width:%6.2fmm", yaw, width);
		
	printf("close device\n");
	HidUart_Close(dev);

	return 0;
}

/* ctrl + c signal handler */
void sigintFunc(int sig)
{
	RSTorqueOnOff(dev,0,1,servoNum);
	printf("Gain OFF.\n");

	printf("close device\n");
	HidUart_Close(dev);

	exit(0);
}

void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num, double base_angle, double base_offset_x, double base_offset_y )
{

	double a,b,tx,ty,lx,ly;		
	double thete1,thete2;		
	double s;					

	if(sign<0) s=-1.0;
	else s=1.0;

	lx= x-base_offset_x;
	ly= y-base_offset_y;
	x = (lx*cos(-base_angle/180.0*M_PI)-ly*sin(-base_angle/180.0*M_PI));
	y = (lx*sin(-base_angle/180.0*M_PI)+ly*cos(-base_angle/180.0*M_PI));

	ty = y;
	tx = x+X_OFS;

	if(hypot(tx,ty)>=AXISLEN_A+AXISLEN_B){

		sPos[0] = (short) (atan2(ty,tx)/M_PI*1800.f);
		sPos[1] = 0;

	}
	else{

		a = acos( (-(tx*tx+ty*ty) + AXISLEN_A*AXISLEN_A + AXISLEN_B*AXISLEN_B) / (2 * AXISLEN_A * AXISLEN_B));
		b = s*acos( (- AXISLEN_A*AXISLEN_A + AXISLEN_B*AXISLEN_B + (tx*tx+ty*ty)) / (2 * AXISLEN_A * sqrt(tx*tx+ty*ty)));

		if(tx<0){
			if(ty>=0) thete1 = M_PI + atan(ty/tx) +b;
			else thete1 = atan(ty/tx) + b - M_PI;
		}
		else thete1 = atan(ty/tx)+b;
		thete2 = -(M_PI-a)*s;

		if(thete1!=0.0) thete1 = thete1 / M_PI * 180.0;
		if(thete2!=0.0) thete2 = thete2 / M_PI * 180.0;

		sPos[0] = (short) (thete1*10.0);
		sPos[1] = (short) (thete2*10.0);


	}
	if(sPos[0]<-ARM_RAD_RANGE) sPos[0]=-ARM_RAD_RANGE;
	else if(sPos[0]>ARM_RAD_RANGE) sPos[0]=ARM_RAD_RANGE;

	if(sPos[1]<-ARM_RAD_RANGE) sPos[1]=-ARM_RAD_RANGE;
	else if(sPos[1]>ARM_RAD_RANGE) sPos[1]=ARM_RAD_RANGE;

	sPos[2] = HEIGHT_TO_RAD(z);

	if(num>3){
		sPos[4] = WIDTH_TO_RAD(w,CROW_POS);

		if(sPos[4]<-HAND_WIDTH_RANGE) sPos[4]=-HAND_WIDTH_RANGE;
		else if(sPos[4]>HAND_WIDTH_RANGE) sPos[4]=HAND_WIDTH_RANGE;

		sPos[3] = -sPos[0]-sPos[1];
		sPos[3] += (short) (yaw*10.0);
	}

}

int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num)
{
	unsigned char	sendbuf[256],*bufp;				
	unsigned char	sum;							
	int				ret;							
	unsigned int	data_len=0,len=0;				
	unsigned char	i;

	memset( sendbuf, 0x00, sizeof( sendbuf ));

	sendbuf[0]  = (unsigned char)0xFA;				
	sendbuf[1]  = (unsigned char)0xAF;				
	sendbuf[2]  = (unsigned char)0x00;				
	sendbuf[3]  = (unsigned char)0x00;				
	sendbuf[4]  = (unsigned char)0x24;				
	sendbuf[5]  = (unsigned char)0x01+1;			
	sendbuf[6]  = (unsigned char)num;				

	data_len = 7;

	bufp = &sendbuf[7];								

	for(i=0;i<num;i++){
		*bufp++ = id+i;								
		data_len++;									

		*bufp++ = (unsigned char)(sMode&0x00FF);	
		data_len++;									
	}

	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						
	data_len++;										

	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );

	if(ret!=HID_UART_SUCCESS) return FALSE;

	return ReadLocalEcho(dev,sendbuf,data_len);
}

int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam)
{
	unsigned char	sendbuf[32];
	unsigned char	readbuf[128];
	unsigned char	sum;
	unsigned int i;
	int				ret;
	unsigned int	len, readlen;
	short			angle;

	memset( sendbuf, 0x00, sizeof( sendbuf ));

	sendbuf[0]  = (unsigned char)0xFA;				
	sendbuf[1]  = (unsigned char)0xAF;				
	sendbuf[2]  = (unsigned char)id;				
	sendbuf[3]  = (unsigned char)0x0f;				
	sendbuf[4]  = (unsigned char)0x2a;				
	sendbuf[5]  = (unsigned char)0x02;				
	sendbuf[6]  = (unsigned char)0x00;				

	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								

	ret = HidUart_Write(dev,sendbuf, 8, &len);

	if(!ReadLocalEcho(dev,sendbuf,len)) return FALSE;

	if( len < 8 ) return FALSE;

	memset( readbuf, 0x00, sizeof( readbuf ));

	readlen = (2) + (1) + (1) + (1) + (1) + (1) + (2) + (1);
	len = 0;

	HidUart_Read(dev,readbuf, readlen, &len);

	if( len < readlen)  return FALSE;

	sum = readbuf[2];
	for( i = 3; i < readlen; i++ ){
		sum = sum ^ readbuf[i];
	}

	if( sum ) return FALSE;

	angle = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);
	if(getParam) *getParam = angle;

	return TRUE;
}

void rad_to_pos(double *x, double *y, double *z, double *yaw, double *w, short *sPos, int num, double base_angle, double base_offset_x, double base_offset_y) {
	double tx, ty;
	double lx, ly;

	ty = sin(SVPOS_TO_RAD(sPos[0])) * AXISLEN_A	+ sin(SVPOS_TO_RAD(sPos[0] + sPos[1])) * AXISLEN_B;
	tx = cos(SVPOS_TO_RAD(sPos[0])) * AXISLEN_A	+ cos(SVPOS_TO_RAD(sPos[0] + sPos[1])) * AXISLEN_B;
	tx -= X_OFS;
	lx = tx;
	ly = ty;
	tx = (lx * cos(base_angle / 180.0 * M_PI) - ly * sin(base_angle / 180.0 * M_PI)) + base_offset_x;
	ty = (lx * sin(base_angle / 180.0 * M_PI) + ly * cos(base_angle / 180.0 * M_PI)) + base_offset_y;
	*x = tx;
	*y = ty;
	*z = RAD_TO_HEIGHT(sPos[2]);
	if (num > 3) {
		*yaw = (double) (sPos[3] + sPos[0] + sPos[1]) / 10.0;
		*w = RAD_TO_WIDTH(sPos[4], CROW_POS);
	}
}

void getPosition(double *x, double *y, double *z, double *yaw, double *width, double base_angle, double base_offset_x, double base_offset_y) {

    // x,y,z,yaw,w;				
	short pos[5] = { 0, 0, 0, 0, 0 };		

	for (int i = 0; i < servoNum; i++) {
		if (!RSGetAngle(dev, i + 1, &pos[i])) {
			printf("Faild to retrive angle of servo %d.\n", i);
		}
	}
	printf("X:%+5d, Y:%+5d, Z:%+5d\n", pos[0], pos[1], pos[2]);
	rad_to_pos(x, y, z, yaw, width, pos, 5, base_angle, base_offset_x, base_offset_y);
    //printf("X:%+.1fmm Y:%+.1fmm Z:%+.1fmm",x,y,z);
    //if(servoNum==5) printf(" Yaw:%+.1fdeg Width:%.1fmm",yaw,width);
    //printf("\n");
}

int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num)
{
	unsigned char	sendbuf[256],*bufp;
	unsigned char	sum;
	unsigned char	i;
	int				ret;
	unsigned int	len,data_len;


	memset( sendbuf, 0x00, sizeof( sendbuf ));

	sendbuf[0]  = (unsigned char)0xFA;				    
	sendbuf[1]  = (unsigned char)0xAF;				    
	sendbuf[2]  = (unsigned char)0;						
	sendbuf[3]  = (unsigned char)0x00;				   
	sendbuf[4]  = (unsigned char)0x1E;				  
	sendbuf[5]  = (unsigned char)0x04+1;			  
	sendbuf[6]  = (unsigned char)num;				  

	data_len = 7;

	bufp = &sendbuf[7];
	for(i=0;i<num;i++){
		*bufp++ = id+i;								
		data_len++;									

		*bufp++ = (unsigned char)(sPoss[i]&0x00FF);
		*bufp++ = (unsigned char)((sPoss[i]&0xFF00)>>8);
		data_len+=2;							

		*bufp++ = (unsigned char)(sTime&0x00FF);
		*bufp++ = (unsigned char)((sTime&0xFF00)>>8);
		data_len+=2;								
	}


	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						
	data_len++;									

	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
	if(ret!=HID_UART_SUCCESS) return FALSE;

	return ReadLocalEcho(dev,sendbuf,data_len);

}


int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,unsigned int data_len)
{

	unsigned char readbuf[1024];
	unsigned int len=0;
	memset(readbuf,0,sizeof(readbuf));

	HidUart_Read( dev, (BYTE*) readbuf, data_len, &len );

	if(data_len!=len) return FALSE;

	for(unsigned long i=0;i<len;i++){
		if(readbuf[i]!=sendbuf[i]) return FALSE;
	}
	return TRUE;
}


int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num)
{
	unsigned char	sendbuf[256],*bufp;
	unsigned char	sum;
	unsigned char	i,j;
	int				ret;
	unsigned int	len,data_len;

	memset( sendbuf, 0x00, sizeof( sendbuf ));

	sendbuf[0]  = (unsigned char)0xFA;				   
	sendbuf[1]  = (unsigned char)0xAF;				   
	sendbuf[2]  = (unsigned char)0;						
	sendbuf[3]  = (unsigned char)0x00;				 
	sendbuf[4]  = (unsigned char)address;				   
	sendbuf[5]  = (unsigned char)size+1;			   
	sendbuf[6]  = (unsigned char)num;				    

	data_len = 7;

	bufp = &sendbuf[7];
	for(i=0;i<num;i++){
		*bufp++ = id+i;								
		data_len++;									

		for(j=0;j<size;j++){
			*bufp++ = (unsigned char)data[j];		
			data_len++;								
		}
	}

	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						
	data_len++;										

	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
	if(ret!=HID_UART_SUCCESS) return FALSE;

	return ReadLocalEcho(dev,sendbuf,data_len);

}

int SetTXOpenDrain(HID_UART_DEVICE dev )
{
	BYTE pinConfig[13];
	BOOL useSuspendValues;
	WORD suspendValue;
	WORD suspendMode;
	BYTE rs485Level;
	BYTE clkDiv;

	HidUart_GetPinConfig(dev,  pinConfig,  &useSuspendValues, &suspendValue, &suspendMode, &rs485Level, &clkDiv);
	pinConfig[10] = 0x01;
	return HidUart_SetPinConfig(dev,  pinConfig, useSuspendValues, suspendValue, suspendMode,rs485Level, clkDiv);


}

