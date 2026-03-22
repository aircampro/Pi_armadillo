/*
    ref :- https://github.com/cbosoft/rheometer/tree/master/src/sensors/encoder
	modified to have 4 encoders attached (counts ground as well so there are 5 pulse counters)
	modified so any encoders reading low are removed (lost wire)
	you could also look at rate of change of each encoder and elimate noisy signals
	this one can be used as a pure encoder rather than with the rheometer
*/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
//#include <pthread.h>
#include <sys/time.h>
#include <wiringPi.h>

//#include "../../util/error.h"
//#include "../../control/control.h"
//#include "../../log/log.h"

#include "encoder.h"

/*
   define interrupt for each of the pins (defined as OPTENC_COUNT) rising and falling edge
*/
#define OPT_MARK(IDX) opt_mark_ ## IDX
#define OPT_SETUP(IDX) \
  if (wiringPiISR(opt_pins[IDX], INT_EDGE_BOTH, &(OPT_MARK(IDX))) < 0) \
    ferr("opt_setup", "failed to set up interrupt for pin %d", opt_pins[0]);

// globals required for optenc operation
static const uint8_t opt_pins[OPTENC_COUNT] = {20, 21, 22, 23, 24};
static struct run_data *ord = NULL;

#define EDIFF 3                                      // this many pulses sees and error between the encoders 
#define PMIN 2                                       // this many pulses means we havent got a connection

void opt_mark(unsigned int i);
void opt_mark_0(void) { opt_mark(0); }                 // GND
void opt_mark_1(void) { opt_mark(1); }
void opt_mark_2(void) { opt_mark(2); }
void opt_mark_3(void) { opt_mark(3); }
void opt_mark_4(void) { opt_mark(4); }

unsigned int tripc[OPTENC_COUNT] = {0};
struct timeval last_convert = {0, 0};
int opt_log_idxs[OPTENC_COUNT] = {0};
static double speed_conversion_timeout = 0.2;
double wire_errors = 0.0f;
unsigned int lst_tripc = 0;

void opt_setup()
{
  char logname[10] = {0};
  for (uint8_t i = 0; i < OPTENC_COUNT; i++) {
    pinMode(opt_pins[i], INPUT);
  }
  OPT_SETUP(0);                                             /* set interrupt to counter encoder on channel GND */
  OPT_SETUP(1);                                             /* set interrupt to counter encoder on channel 1 */
  OPT_SETUP(2);                                             /* set interrupt to counter encoder on channel 2 */
  OPT_SETUP(3);                                             /* set interrupt to counter encoder on channel 3 */
  OPT_SETUP(4);                                             /* set interrupt to counter encoder on channel 4 */
  gettimeofday(&last_convert, NULL);
  sleep(1);

}

/*
   count each interrupt for each of the encoders 
*/
void opt_mark(unsigned int i)
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  tripc[i] ++;
}

/*
   function to calculate the speed of each encoder
*/
double measure_speed()
{
  struct timeval now = {0, 0};
  gettimeofday(&now, NULL);
  double dseconds = ((double)now.tv_sec) - ((double)last_convert.tv_sec);
  double duseconds = ((double)now.tv_usec) - ((double)last_convert.tv_usec);
  double dt = dseconds + (duseconds*0.001*0.001);
  static double last_speed = 0.0;

    last_convert = now;
    int count = 0;
    for (int i = 0; i < OPTENC_COUNT; i++) {

#ifdef DEBUG
      count += 3;
#else
      if (i != 0) {
		  if (((tripc[i] - lst_tripc) < -EDIFF) && (tripc[i] < PMIN)) {                      // this channel low
			wire_errors = wire_errors + 1.0f;  
          } else if ((((tripc[i] - lst_tripc) > EDIFF) && (lst_tripc < PMIN)) && (i==1)) {   // first channel low
			count = count - lst_tripc;                                                       // remove first counter
            count += tripc[i];                                                               // add current	
			wire_errors = wire_errors + 1.0f;  	                                             // increase errors 		
          } else {
            count += tripc[i];			  
          }
      } else {
          count += tripc[i];
      }
      lst_tripc = tripc[i];
      tripc[i] = 0;
#endif

    }

    double dtripc = ((double)count) / (((double)OPTENC_COUNT) - wire_errors);               // average over the OPTENC_COUNT number of encoders 
    last_speed = dtripc / dt;                                                               // returns speed in hz (rev. per second)

  if (wire_errors > 0.0f) {
      printf("error with encoder wire connections\n");
  }
  wire_errors = 0.0f;
  return last_speed;
}

int main(void) {
	double spd = 0.0f;
    opt_setup();
    for (int n=0; n < 10000; n++){                                 // show for 10000 iterations
		spd = measure_speed();
		printf("speed %f rpm \n", spd);
	}		
	return 0;
}