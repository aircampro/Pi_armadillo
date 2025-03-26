/*

ref :- is raspicade joystick development it has been modified to control the tellio drone

https://jammastar.com/gb/joysticks-kits/841-kit-zyppyy-2-players-16-buttons-xin-mo-usb-encoder.html
https://github.com/raspicade
https://github.com/raspicade/Recalbox-Retrogame-2Players-Pi2
https://github.com/raspicade/Raspicade-kernel-xinmo/tree/master
https://wiki.yoctoproject.org/wiki/How_do_I
https://github.com/sercr0388
https://github.com/sercr0388/komm/tree/master/komm


ADAFRUIT RETROGAME UTILITY: remaps buttons on Raspberry Pi GPIO header
to virtual USB keyboard presses.  Great for classic game emulators!
Retrogame is interrupt-driven and efficient (usually under 0.3% CPU use)
and debounces inputs for glitch-free gaming.

Connect one side of button(s) to GND pin (there are several on the GPIO
header, but see later notes) and the other side to GPIO pin of interest.
Internal pullups are used; no resistors required.  Avoid pins 8 and 10;
these are configured as a serial port by default on most systems (this
can be disabled but takes some doing).  Pin configuration is currently
set in global table; no config file yet.  See later comments.

Must be run as root, i.e. 'sudo ./retrogame &' or configure init scripts
to launch automatically at system startup.

Requires uinput kernel module.  This is typically present on popular
Raspberry Pi Linux distributions but not enabled on some older varieties.
To enable, either type:

    sudo modprobe uinput

Or, to make this persistent between reboots, add a line to /etc/modules:

    uinput

Prior versions of this code, when being compiled for use with the Cupcade
or PiGRRL projects, required CUPCADE to be #defined.  This is no longer
the case; instead a test is performed to see if a PiTFT is connected, and
one of two I/O tables is automatically selected.

Written by Phil Burgess for Adafruit Industries, distributed under BSD
License.  Adafruit invests time and resources providing this open source
code, please support Adafruit and open-source hardware by purchasing
products from Adafruit!


Copyright (c) 2013 Adafruit Industries.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <linux/input.h>
#include <linux/uinput.h>

// Socket Library
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
// OpenCV Library
#include <opencv2/opencv.hpp>
// Used for standby processing
#include <thread>
#include <chrono>

/* Tellio Drone Commands 

Initial Commands
Command Name	explanation
command	Start SDK mode (always run first when remotely controlling with commands)
streamon	Enable Video Stream (Start Sending Camera Footage)
streamoff	Disable video stream (stop transmitting camera footage)
emergency	Emergency stop (stop the motor immediately)
wifi ssid pass	Set the Wi-Fi SSID and password (can also be set from the Tello app)
ap ssid pass	Set Tello to station mode and connect to a new access point using the access point's SSID and password

Control Commands
Command Name	explanation
takeoff	takeoff
land	landing
stop	Stop in mid-air (hovering)
speed x	Set speed to x[cm/s] (value range: 10~100)
left x	Move x[cm] to the left (value range: 20~500)
right x	Move x[cm] to the right (value range: 20~500)
forward x	Move forward x[cm] (value range: 20~500)
back x	Move back x[cm] (value range: 20~500)
cw x	Clockwise x[deg] rotation (value range: 1~3600)
cow x	Rotate x[deg] counterclockwise (value range: 1~3600)
flip x	Flip in X direction (L/R/F/B)

Get Status Commands
Command Name	explanation
battery?	Get battery level (percentage)
speed?	Get the current flight speed (cm/s)
time?	Get cumulative flight time (seconds)
height?	Get Current Altitude (cm)
temp?	Get the lowest and highest temperature (°C)
attitude?	Get pitch, roll, and yaw angles
baro?	Get the current barometric pressure (hPa)
acceleration?	Acceleration along the x,y,z axis (0.001G)
tof?	Time-of-Flight (distance sensor) altitude (cm) acquisition
wifi?	Get Wi-Fi signal strength (dBm)

*/

// START HERE ------------------------------------------------------------
// This table remaps GPIO inputs to keyboard values.  In this initial
// implementation there's a 1:1 relationship (can't attach multiple keys
// to a button) and the list is fixed in code; there is no configuration
// file.  Buttons physically connect between GPIO pins and ground.  There
// are only a few GND pins on the GPIO header, so a breakout board is
// often needed.  If you require just a couple extra ground connections
// and have unused GPIO pins, set the corresponding key value to GND to
// create a spare ground point.

#define GND -1
struct {
	int pin;
	int key;
} *io, // In main() this pointer is set to one of the two tables below.
   ioStandard[] = {
	// This pin/key table is used if an Adafruit PiTFT display
	// is detected (e.g. Cupcade or PiGRRL).
	// Input   Output (from /usr/include/linux/input.h)
	//GPIO Mapping
	{  2,     KEY_0       },   	// HotKey Player1
	{  3,     KEY_9     },    	// HotKey Player2
	{  4,     KEY_UP     },   	// UP Player1
	{ 17,     KEY_DOWN    },   	// DOWN Player1
	{ 27,     KEY_LEFT },   	// LEFT Player1
	{ 22,     KEY_RIGHT  },  	// RIGHT Player1
	{ 10,     KEY_1  },    		// START Player1
	{  9,     KEY_5  },   		// COIN/SELECT Player1
	{ 11,     KEY_R  },   		// UP Player2
	{  5,     KEY_F  },   		// DOWN Player2
	{  6,     KEY_D  },   		// LEFT Player2
	{ 13,     KEY_G  },   		// RIGHT Player2
	{ 19,     KEY_2  },   		// START Player2
	{ 26,     KEY_6  },   		// COIN/SELECT Player2
	{ 14,     KEY_Z  },   		// BUTTON5/TL Player1
	{ 15,     KEY_SPACE  },   	// BUTTON2/X Player1
	{ 18,     KEY_LEFTSHIFT },  // BUTTON4/Y Player1
	{  23,    KEY_X     },      // BUTTON6/TR Player1
	{  24,    KEY_LEFTCTRL  },  // BUTTON1/A Player1
	{  25,    KEY_LEFTALT   },  // BUTTON2/B Player1
	{   8,    KEY_E },     		// BUTTON5/TL Player2
	{   7,    KEY_Q  },   		// BUTTON3/X Player2
	{  12,    KEY_S  },     	// BUTTON2/Y Player2
	{  16,    KEY_T  },   		// BUTTON6/TR Player2
	{  20,    KEY_A  },   		// BUTTON1/A Player2
	{  21,    KEY_S  },   		// BUTTON2/B Player2
	{  -1,     -1           } }; // END OF LIST, DO NOT CHANGE

	// MAME must be configured with 'z' & 'x' as buttons 1 & 2 -
	// this was required for the accompanying 'menu' utility to
	// work (catching crtl/alt w/ncurses gets totally NASTY).
	// Credit/start are likewise moved to 'r' & 'q,' reason being
	// to play nicer with certain emulators not liking numbers.
	// GPIO options are 'maxed out' with PiTFT + above table.
	// If additional buttons are desired, will need to disable
	// serial console and/or use P5 header.  Or use keyboard.
/*   ioTFT[] = {
	// This pin/key table is used when the PiTFT isn't found
	// (using HDMI or composite instead), as with our original
	// retro gaming guide.
	// Input   Output (from /usr/include/linux/input.h)
	{  25,     KEY_LEFT     },   // Joystick (4 pins)
	{   9,     KEY_RIGHT    },
	{  10,     KEY_UP       },
	{  17,     KEY_DOWN     },
	{  23,     KEY_LEFTCTRL },   // A/Fire/jump/primary
	{   7,     KEY_LEFTALT  },   // B/Bomb/secondary
	// For credit/start/etc., use USB keyboard or add more buttons.
	{  -1,     -1           } }; // END OF LIST, DO NOT CHANGE
*/
// A "Vulcan nerve pinch" (holding down a specific button combination
// for a few seconds) issues an 'esc' keypress to MAME (which brings up
// an exit menu or quits the current game).  The button combo is
// configured with a bitmask corresponding to elements in the above io[]
// array.  The default value here uses elements 6 and 7 (credit and start
// in the Cupcade pinout).  If you change this, make certain it's a combo
// that's not likely to occur during actual gameplay (i.e. avoid using
// joystick directions or hold-for-rapid-fire buttons).
// Also key auto-repeat times are set here.  This is for navigating the
// game menu using the 'gamera' utility; MAME disregards key repeat
// events (as it should).
const unsigned long vulcanMask = (1L << 6) | (1L << 7);
const int           vulcanKey  = KEY_ESC, // Keycode to send
                    vulcanTime = 1000,    // Pinch time in milliseconds
                    repTime1   = 500,     // Key hold time to begin repeat
                    repTime2   = 100;     // Time between key repetitions

// A few globals ---------------------------------------------------------

char
  *progName,                         // Program name (for error reporting)
   sysfs_root[] = "/sys/class/gpio", // Location of Sysfs GPIO files
   running      = 1;                 // Signal handler will set to 0 (exit)
volatile unsigned int  *gpio;       // GPIO register table
const int
   debounceTime = 20;                // 20 ms for button debouncing


// Some utility functions ------------------------------------------------

// Set one GPIO pin attribute through the Sysfs interface.
int pinConfig(int pin, char *attr, char *value) {
	char filename[50];
	int  fd, w, len = strlen(value);
	sprintf(filename, "%s/gpio%d/%s", sysfs_root, pin, attr);
	if((fd = open(filename, O_WRONLY)) < 0) return -1;
	w = write(fd, value, len);
	close(fd);
	return (w != len); // 0 = success
}

// Un-export any Sysfs pins used; don't leave filesystem cruft.  Also
// restores any GND pins to inputs.  Write errors are ignored as pins
// may be in a partially-initialized state.
void cleanup() {
	char buf[50];
	int  fd, i;
	sprintf(buf, "%s/unexport", sysfs_root);
	if((fd = open(buf, O_WRONLY)) >= 0) {
		for(i=0; io[i].pin >= 0; i++) {
			// Restore GND items to inputs
			if(io[i].key == GND)
				pinConfig(io[i].pin, "direction", "in");
			// And un-export all items regardless
			sprintf(buf, "%d", io[i].pin);
			write(fd, buf, strlen(buf));
		}
		close(fd);
	}
}

// Quick-n-dirty error reporter; print message, clean up and exit.
void err(char *msg) {
	printf("%s: %s.  Try 'sudo %s'.\n", progName, msg, progName);
	cleanup();
	exit(1);
}

// Interrupt handler -- set global flag to abort main loop.
void signalHandler(int n) {
	running = 0;
}

// Detect Pi board type.  Doesn't return super-granular details,
// just the most basic distinction needed for GPIO compatibility:
// 0: Pi 1 Model B revision 1
// 1: Pi 1 Model B revision 2, Model A, Model B+, Model A+
// 2: Pi 2 Model B

static int boardType(void) {
	FILE *fp;
	char  buf[1024], *ptr;
	int   n, board = 1; // Assume Pi1 Rev2 by default

	// Relies on info in /proc/cmdline.  If this becomes unreliable
	// in the future, alt code below uses /proc/cpuinfo if any better.
#if 1
	if((fp = fopen("/proc/cmdline", "r"))) {
		while(fgets(buf, sizeof(buf), fp)) {
			if((ptr = strstr(buf, "mem_size=")) &&
			   (sscanf(&ptr[9], "%x", &n) == 1) &&
			   (n == 0x3F000000)) {
				board = 2; // Appears to be a Pi 2
				break;
			} else if((ptr = strstr(buf, "boardrev=")) &&
			          (sscanf(&ptr[9], "%x", &n) == 1) &&
			          ((n == 0x02) || (n == 0x03))) {
				board = 0; // Appears to be an early Pi
				break;
			}
		}
		fclose(fp);
	}
#else
	char s[8];
	if((fp = fopen("/proc/cpuinfo", "r"))) {
		while(fgets(buf, sizeof(buf), fp)) {
			if((ptr = strstr(buf, "Hardware")) &&
			   (sscanf(&ptr[8], " : %7s", s) == 1) &&
			   (!strcmp(s, "BCM2709"))) {
				board = 2; // Appears to be a Pi 2
				break;
			} else if((ptr = strstr(buf, "Revision")) &&
			          (sscanf(&ptr[8], " : %x", &n) == 1) &&
			          ((n == 0x02) || (n == 0x03))) {
				board = 0; // Appears to be an early Pi
				break;
			}
		}
		fclose(fp);
	}
#endif

	return board;
}

// Main stuff ------------------------------------------------------------

#define PI1_BCM2708_PERI_BASE 0x20000000
#define PI1_GPIO_BASE         (PI1_BCM2708_PERI_BASE + 0x200000)
#define PI2_BCM2708_PERI_BASE 0x3F000000
#define PI2_GPIO_BASE         (PI2_BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE            (4*1024)
#define GPPUD                 (0x94 / 4)
#define GPPUDCLK0             (0x98 / 4)

int main(int argc, char *argv[]) {

	// A few arrays here are declared with 32 elements, even though
	// values aren't needed for io[] members where the 'key' value is
	// GND.  This simplifies the code a bit -- no need for mallocs and
	// tests to create these arrays -- but may waste a handful of
	// bytes for any declared GNDs.
	char                   buf[50],      // For sundry filenames
	                       c,            // Pin input value ('0'/'1')
	                       board;        // 0=Pi1Rev1, 1=Pi1Rev2, 2=Pi2
	int                    fd,           // For mmap, sysfs, uinput
	                       i, j,         // Asst. counter
	                       bitmask,      // Pullup enable bitmask
	                       timeout = -1, // poll() timeout
	                       intstate[32], // Last-read state
	                       extstate[32], // Debounced state
	                       lastKey = -1; // Last key down (for repeat)
	unsigned long          bitMask, bit; // For Vulcan pinch detect
	volatile unsigned char shortWait;    // Delay counter
	struct input_event     keyEv, synEv; // uinput events
	struct pollfd          p[32];        // GPIO file descriptors

	progName = argv[0];             // For error reporting
	signal(SIGINT , signalHandler); // Trap basic signals (exit cleanly)
	signal(SIGKILL, signalHandler);

//not needed
/*
	// Select io[] table for Cupcade (TFT) or 'normal' project.
	io = (access("/etc/modprobe.d/adafruit.conf", F_OK) ||
	      access("/dev/fb1", F_OK)) ? ioStandard : ioTFT;
*/
    // reads in the interface map for the joystick wiring loom
	io = ioStandard;
	// If this is a "Revision 1" Pi board (no mounting holes),
	// remap certain pin numbers in the io[] array for compatibility.
	// This way the code doesn't need modification for old boards.
	board = boardType();
	if(board == 0) {
		for(i=0; io[i].pin >= 0; i++) {
			if(     io[i].pin ==  2) io[i].pin = 0;
			else if(io[i].pin ==  3) io[i].pin = 1;
			else if(io[i].pin == 27) io[i].pin = 21;
		}
	}

	// ----------------------------------------------------------------
	// Although Sysfs provides solid GPIO interrupt handling, there's
	// no interface to the internal pull-up resistors (this is by
	// design, being a hardware-dependent feature).  It's necessary to
	// grapple with the GPIO configuration registers directly to enable
	// the pull-ups.  Based on GPIO example code by Dom and Gert van
	// Loo on elinux.org

	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
		err("Can't open /dev/mem");
	gpio = mmap(            // Memory-mapped I/O
	  NULL,                 // Any adddress will do
	  BLOCK_SIZE,           // Mapped block length
	  PROT_READ|PROT_WRITE, // Enable read+write
	  MAP_SHARED,           // Shared with other processes
	  fd,                   // File to map
	  (board == 2) ?
	   PI2_GPIO_BASE :      // -> GPIO registers
	   PI1_GPIO_BASE);

	close(fd);              // Not needed after mmap()
	if(gpio == MAP_FAILED) err("Can't mmap()");
	// Make combined bitmap of pullup-enabled pins:
	for(bitmask=i=0; io[i].pin >= 0; i++)
		if(io[i].key != GND) bitmask |= (1 << io[i].pin);
	gpio[GPPUD]     = 2;                    // Enable pullup
	for(shortWait=150;--shortWait;);        // Min 150 cycle wait
	gpio[GPPUDCLK0] = bitmask;              // Set pullup mask
	for(shortWait=150;--shortWait;);        // Wait again
	gpio[GPPUD]     = 0;                    // Reset pullup registers
	gpio[GPPUDCLK0] = 0;
	(void)munmap((void *)gpio, BLOCK_SIZE); // Done with GPIO mmap()


	// ----------------------------------------------------------------
	// All other GPIO config is handled through the sysfs interface.

	sprintf(buf, "%s/export", sysfs_root);
	if((fd = open(buf, O_WRONLY)) < 0) // Open Sysfs export file
		err("Can't open GPIO export file");
	for(i=j=0; io[i].pin >= 0; i++) { // For each pin of interest...
		sprintf(buf, "%d", io[i].pin);
		write(fd, buf, strlen(buf));             // Export pin
		pinConfig(io[i].pin, "active_low", "0"); // Don't invert
		if(io[i].key == GND) {
			// Set pin to output, value 0 (ground)
			if(pinConfig(io[i].pin, "direction", "out") ||
			   pinConfig(io[i].pin, "value"    , "0"))
				err("Pin config failed (GND)");
		} else {
			// Set pin to input, detect rise+fall events
			if(pinConfig(io[i].pin, "direction", "in") ||
			   pinConfig(io[i].pin, "edge"     , "both"))
				err("Pin config failed");
			// Get initial pin value
			sprintf(buf, "%s/gpio%d/value",
			  sysfs_root, io[i].pin);
			// The p[] file descriptor array isn't necessarily
			// aligned with the io[] array.  GND keys in the
			// latter are skipped, but p[] requires contiguous
			// entries for poll().  So the pins to monitor are
			// at the head of p[], and there may be unused
			// elements at the end for each GND.  Same applies
			// to the intstate[] and extstate[] arrays.
			if((p[j].fd = open(buf, O_RDONLY)) < 0)
				err("Can't access pin value");
			intstate[j] = 0;
			if((read(p[j].fd, &c, 1) == 1) && (c == '0'))
				intstate[j] = 1;
			extstate[j] = intstate[j];
			p[j].events  = POLLPRI; // Set up poll() events
			p[j].revents = 0;
			j++;
		}
	} // 'j' is now count of non-GND items in io[] table
	close(fd); // Done exporting


	// ----------------------------------------------------------------
	// Set up uinput

#if 1
	// Retrogame normally uses /dev/uinput for generating key events.
	// Cupcade requires this and it's the default.  SDL2 (used by
	// some newer emulators) doesn't like it, wants /dev/input/event0
	// instead.  Enable that code by changing to "#if 0" above.
	if((fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK)) < 0)
		err("Can't open /dev/uinput");
	if(ioctl(fd, UI_SET_EVBIT, EV_KEY) < 0)
		err("Can't SET_EVBIT");
	for(i=0; io[i].pin >= 0; i++) {
		if(io[i].key != GND) {
			if(ioctl(fd, UI_SET_KEYBIT, io[i].key) < 0)
				err("Can't SET_KEYBIT");
		}
	}
	if(ioctl(fd, UI_SET_KEYBIT, vulcanKey) < 0) err("Can't SET_KEYBIT");
	struct uinput_user_dev uidev;
	memset(&uidev, 0, sizeof(uidev));
	snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "retrogame");
	uidev.id.bustype = BUS_USB;
	uidev.id.vendor  = 0x1;
	uidev.id.product = 0x1;
	uidev.id.version = 1;
	if(write(fd, &uidev, sizeof(uidev)) < 0)
		err("write failed");
	if(ioctl(fd, UI_DEV_CREATE) < 0)
		err("DEV_CREATE failed");
#else // SDL2 prefers this event methodology
	if((fd = open("/dev/input/event0", O_WRONLY | O_NONBLOCK)) < 0)
		err("Can't open /dev/input/event0");
#endif

    // socket communication to the drone
    int socket_desc;
    struct sockaddr_in server;
    const char *command;

    // Create UDP Socket for communication to the drone
    socket_desc = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_desc == -1) {
        printf("Failed to create socket.");
        return 1;
    }

    // Prepare the sockaddr_in structure
    server.sin_addr.s_addr = inet_addr("192.168.10.1");                        // IP Address of the server
    server.sin_family = AF_INET;                                               // IPv4 Protocol If you want to use IPv6, you can use AF_INET6
    server.sin_port = htons(8889);                                             // Port

    const int in_air = 1;
    const int on_ground = 0;
    int drone_state = on_ground;
    float drone_speed = 20;                                                      // Set speed to x[cm/s] (value range: 10~100)  
    bool speed_change = false;
	
	// Initialize input event structures
	memset(&keyEv, 0, sizeof(keyEv));
	keyEv.type  = EV_KEY;
	memset(&synEv, 0, sizeof(synEv));
	synEv.type  = EV_SYN;
	synEv.code  = SYN_REPORT;
	synEv.value = 0;

	// 'fd' is now open file descriptor for issuing uinput events

    // set video stream to be on and open it.
    command = "streamon";
    if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
        std::cout << "Failed to start video stream." << std::endl;
        return 1;
    } else {
        std::string telloStreamAddress = "udp://0.0.0.0:11111";
		// Wait 1 seconds
        std::this_thread::sleep_for(std::chrono::seconds(1));
        cv::VideoCapture cap(telloStreamAddress);

        if (!cap.isOpened()) {
            std::cout << "Error: Cloud not open video stream." << std::endl;
            return 1;
        }
    }
    cv::Mat frame;
    int edge_det_sel = 0;                                                                                   // if true to video edge detection
	cv::Mat img_c, img_l, img_s_x, img_s_y, img_s;
    bool save_frames = false;
	
	// set speed to nominal speed
    sprintf(command, "speed %f", drone_speed);
    if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
        std::cout << "Failed to send speed command." << std::endl;
    } 
		
	// ----------------------------------------------------------------
	// Monitor GPIO file descriptors for button events.  The poll()
	// function watches for GPIO IRQs in this case; it is NOT
	// continually polling the pins!  Processor load is near zero.

	while(running) { // Signal handler can set this to 0 to exit
        cap >> frame;                                                                     // show video stream
        if (frame.empty()) {
            printf("Error: Received empty frame.");
            continue;
        }
        switch(edge_det_sel) {
            case 1: 
		    cv::Canny(frame, img_c, 125, 255);
            cv::imshow("Tello Stream Canny", img_c);			
		    break;
			
			case 2:
            cv::Laplacian(frame, img_l, 3);
	        cv::convertScaleAbs(img_l, img_l, 1, 0);
	        cv::threshold(img_l, img_l, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
            cv::imshow("Tello Stream Laplace", img_l);	
            break;
			
			case 3:
	        cv::Sobel(frame, img_s_x, CV_8UC1, 1, 0, 3);
	        cv::Sobel(frame, img_s_y, CV_8UC1, 0, 1, 3);
	        img_s = abs(img_s_x) + abs(img_s_y);
	        cv::convertScaleAbs(img_s, img_s, 1, 0);
	        cv::threshold(img_s, img_s, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
            cv::imshow("Tello Stream Sobel", img_s);	
            break;
			
			case 0:
            cv::imshow("Tello Stream", frame);
		    break;
        }

        if (save_frames == true) {
	        cv::imwrite("normal.jpg", frame);
	        cv::imwrite("canny.jpg", img_c);
	        cv::imwrite("laplace.jpg", img_l);
	        cv::imwrite("sobel.jpg", img_s);
            save_frames = false;
			std::cout << "frames we saved .... " << std::endl;
        }
		
		// Wait for IRQ on pin (or timeout for button debounce)
		if(poll(p, j, timeout) > 0) { // If IRQ...
			for(i=0; i<j; i++) {       // Scan non-GND pins...
				if(p[i].revents) { // Event received?
					// Read current pin state, store
					// in internal state flag, but
					// don't issue to uinput yet --
					// must wait for debounce!
					lseek(p[i].fd, 0, SEEK_SET);
					read(p[i].fd, &c, 1);
					if(c == '0')      intstate[i] = 1;
					else if(c == '1') intstate[i] = 0;
					p[i].revents = 0; // Clear flag
				}
			}
			timeout = debounceTime; // Set timeout for debounce
			c       = 0;            // Don't issue SYN event
			// Else timeout occurred
		} else if(timeout == debounceTime) {                                                               // Button debounce timeout
			// 'j' (number of non-GNDs) is re-counted as
			// it's easier than maintaining an additional
			// remapping table or a duplicate key[] list.
			bitMask = 0L; // Mask of buttons currently pressed
			bit     = 1L;
			for(c=i=j=0; io[i].pin >= 0; i++, bit<<=1) {
				if(io[i].key != GND) {
					// Compare internal state against
					// previously-issued value.  Send
					// keystrokes only for changed states.
					if(intstate[j] != extstate[j]) {
						extstate[j] = intstate[j];
						keyEv.code  = io[i].key;
						keyEv.value = intstate[j];
						/*
							{  4,     KEY_UP     },   	// UP Player1
	                        { 17,     KEY_DOWN    },   	// DOWN Player1
	                        { 27,     KEY_LEFT },   	// LEFT Player1
	                        { 22,     KEY_RIGHT  },  	// RIGHT Player1
							
	                        { 11,     KEY_R  },   		// UP Player2
	                        {  5,     KEY_F  },   		// DOWN Player2
	                        {  6,     KEY_D  },   		// LEFT Player2
	                        { 13,     KEY_G  },   		// RIGHT Player2
                            
	                        { 10,     KEY_1  },    		// START Player1
	                        {  9,     KEY_5  },   		// COIN/SELECT Player1
							
	                        { 14,     KEY_Z  },   		// BUTTON5/TL Player1
	                        { 15,     KEY_SPACE  },   	// BUTTON2/X Player1
	                        { 18,     KEY_LEFTSHIFT },  // BUTTON4/Y Player1
	                        {  23,    KEY_X     },      // BUTTON6/TR Player1
	                        {  24,    KEY_LEFTCTRL  },  // BUTTON1/A Player1
	                        {  25,    KEY_LEFTALT   },  // BUTTON2/B Player1
							
	                        {   8,    KEY_E },     		// BUTTON5/TL Player2
	                        {   7,    KEY_Q  },   		// BUTTON3/X Player2
	                        {  12,    KEY_S  },     	// BUTTON2/Y Player2
	                        {  16,    KEY_T  },   		// BUTTON6/TR Player2
	                        {  20,    KEY_A  },   		// BUTTON1/A Player2
	                        {  21,    KEY_S  },   		// BUTTON2/B Player2
	                    */
                        if ((keyEv.code==KEY_UP)&&(keyEv.value==1)) {
                            command = "forward 20";                                         // forward x x=20-500 cm
                            if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                std::cout << "Failed to send forward 20 cm command." << std::endl;
                            } 
                        } else if ((keyEv.code==KEY_DOWN)&&(keyEv.value==1)) {		
                            command = "back 20";                                           // back x x=20-500 cm
                            if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                std::cout << "Failed to send back 20 cm command." << std::endl;
                            } 
                        } else if ((keyEv.code==KEY_LEFTCTRL )&&(keyEv.value==1)) {	
                            command = "stop";                                            // left x x=20-500 cm
                            if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                std::cout << "Failed to send hover command." << std::endl;
                            } 	
                        } else if ((keyEv.code==KEY_LEFT)&&(keyEv.value==1)) {	
                            command = "left 20";                                          // left x x=20-500 cm
                            if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                std::cout << "Failed to send left 20 cm command." << std::endl;
                            } 						
                        } else if ((keyEv.code==KEY_RIGHT)&&(keyEv.value==1)) {	
                            command = "right 20";                                         // right x x=20-500 cm
                            if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                std::cout << "Failed to send right 20cm command." << std::endl;
                            } 		
                        } else if ((keyEv.code==KEY_Z)&&(keyEv.value==1)) {	
                            command = "cw 10";                                            // right x x=20-500 cm
                            if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                std::cout << "Failed to send turn clockwise 10cm command." << std::endl;
                            } 	
                        } else if ((keyEv.code==KEY_X)&&(keyEv.value==1)) {	
                            command = "ccw 10";                                           // right x x=20-500 cm
                            if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                std::cout << "Failed to send turn counter clockwise 10cm command." << std::endl;
                            } 
                        } else if ((keyEv.code==KEY_LEFTSHIFT)&&(keyEv.value==1)) {	      // speed up
                            drone_speed += 5.0;						
                            speed_change = true; 	
                        } else if ((keyEv.code==KEY_SPACE)&&(keyEv.value==1)) {	          // speed down
                            drone_speed -= 5.0;						
                            speed_change = true;	
                        } else if ((keyEv.code==KEY_E)&&(keyEv.value==1)) {	      // toggle edge detection on video frame 
                            edge_det_sel = 1;	
                        } else if ((keyEv.code==KEY_Q)&&(keyEv.value==1)) {	      // toggle edge detection on video frame 
                            edge_det_sel = 2;	
                        } else if ((keyEv.code==KEY_T)&&(keyEv.value==1)) {	      // toggle edge detection on video frame 
                            edge_det_sel = 3;		
                        } else if ((keyEv.code==KEY_A)&&(keyEv.value==1)) {	      // toggle edge detection on video frame 
                            edge_det_sel = 0;		
                        } else if ((keyEv.code==KEY_LEFTALT)&&(keyEv.value==1)) {	      // save each video frame 
                            save_frames = true;							
                        } else if ((keyEv.code==KEY_1)&&(keyEv.value==1)) {
                            if (drone_state == on_ground) {
                                command = "takeoff";
                                if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                    std::cout << "Failed to send takeoff command." << std::endl;
                                } else {
                                    drone_state = in_air;
                                }
                            }
                        } else if ((keyEv.code==KEY_5)&&(keyEv.value==1)) {		
                            if (drone_state == in_air) {
                                command = "land";
                                if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                    std::cout << "Failed to send land command." << std::endl;
                                } else {
                                    drone_state = on_ground;
                                }
                            }						
                        }
                        if (speed_change == true) {  
                            sprintf(command, "speed %f", drone_speed);							
                            if (sendto(socket_desc, command, strlen(command), 0, (struct sockaddr *)&server, sizeof(server)) < 0) {
                                std::cout << "Failed to send speed command." << std::endl;
                            } else {
                               speed_change == false;
                            }							   
						}							
						//if ((keyEv.code==KEY_0)&&(keyEv.value==1))
						//{
						//	system("sudo halt");
							//system("echo \"that works\"");
						//}
						//else
						//{
							write(fd, &keyEv,
							sizeof(keyEv));
						//}
						//write(fd, &keyEv,
						//sizeof(keyEv));
						c = 1; // Follow w/SYN event
						if(intstate[j]) { // Press?
							// Note pressed key
							// and set initial
							// repeat interval.
							lastKey = i;
							timeout = repTime1;
						} else { // Release?
							// Stop repeat and
							// return to normal
							// IRQ monitoring
							// (no timeout).
							lastKey = timeout = -1;
						}
					}
					j++;
					if(intstate[i]) bitMask |= bit;
				}
			}

			// If the "Vulcan nerve pinch" buttons are pressed,
			// set long timeout -- if this time elapses without
			// a button state change, esc keypress will be sent.
			if((bitMask & vulcanMask) == vulcanMask)
				timeout = vulcanTime;
		} else if(timeout == vulcanTime) { // Vulcan timeout occurred
			// Send keycode (MAME exits or displays exit menu)
			keyEv.code = vulcanKey;
			for(i=1; i>= 0; i--) { // Press, release
				keyEv.value = i;
				write(fd, &keyEv, sizeof(keyEv));
				usleep(10000); // Be slow, else MAME flakes
				write(fd, &synEv, sizeof(synEv));
				usleep(10000);
			}
			timeout = -1; // Return to normal processing
			c       = 0;  // No add'l SYN required
		} else if(lastKey >= 0) { // Else key repeat timeout
			if(timeout == repTime1) timeout = repTime2;
			else if(timeout > 30)   timeout -= 5; // Accelerate
			c           = 1; // Follow w/SYN event
			keyEv.code  = io[lastKey].key;
			keyEv.value = 2; // Key repeat event
			write(fd, &keyEv, sizeof(keyEv));
		}
		if(c) write(fd, &synEv, sizeof(synEv));
	}

	// ----------------------------------------------------------------
	// Clean up

	ioctl(fd, UI_DEV_DESTROY); // Destroy and
	close(fd);                 // close uinput
	cleanup();                 // Un-export pins

    // Close the socket
    close(socket_desc);
	
	puts("Done.");

	return 0;