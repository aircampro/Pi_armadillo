// misra version
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "i2c_scd30_misra.h"
#define use_if_method

int main(int argc, char **argv)
{
        char *bus = "/dev/i2c-3";
        int ret_val = 0;
        float readout[3] = {};

#ifdef use_if_method 		
        // ---------------------- initialise -----------------------------		
        // Set measurement interval (second)
        if (scd30_set_value_misra(bus, 0x4600, 2) == 0)
		{
            // Set Forced Recalibration value (ppm)
           if (scd30_set_value_misra(bus, 0x5204, 450) == 0)
		   {
               // Set Temperature Offset (0.01K)
               if (scd30_set_value_misra(bus, 0x5403, 200) == 0)
			   {
                   // Activate Automatic Self-Calibration
                   if (scd30_set_value_misra(bus, 0x5306, 1) == 0)
				   {
                       // Altitude Compensation (meter)
                       if (scd30_set_value_misra(bus, 0x5102, 30) == 0)
					   {
                            // Soft Reset
                            ret_val = scd30_reset_misra(bus);
                       }
					}
					else
		            {
			           ret_val = 1;
		            }
				}
			    else
		        {
			        ret_val = 1;
		        }
			}
		    else
		    {
			    ret_val = 1;
		    }
		}
		else
		{
			ret_val = 1;
		}
#else
        // ---------- alternative to the above
        ret_val = scd30_set_value_misra(bus, 0x4600, 2);
        // Set Forced Recalibration value (ppm)
        ret_val |= ((scd30_set_value_misra(bus, 0x5204, 450) & 0x3) << 2);
        // Set Temperature Offset (0.01K)
        ret_val |= ((scd30_set_value_misra(bus, 0x5403, 200) & 0x3) << 4);
        // Activate Automatic Self-Calibration
        ret_val |= ((scd30_set_value_misra(bus, 0x5306, 1) & 0x3) << 6);
        // Altitude Compensation (meter)
        ret_val |= ((scd30_set_value_misra(bus, 0x5102, 30) & 0x3) << 8);
        // Soft Reset
        ret_val |= ((scd30_reset_misra(bus) & 0x3) << 10);
#endif
	
        // ---------------------- main loop forever -----------------------------				
        while (ret_val == 0)
        {
                while (scd30_get_data_ready_status_misra(bus) == 2)
                {
                        //wait for ready
                        usleep(10000);
                }

                if (scd30_read_measurement_misra(bus, readout))
                {
                        fprintf(stderr, "Error scd30_read_measurement\n");
                }
                else
                {
                        printf("CO2: %5.f [ppm]  Temp: %4.1f [deg]  Humidity: %3.f [%%RH]\n",
                               readout[0], readout[1], readout[2]);
                }

                usleep(2060000);
        }
        return ret_val;
}