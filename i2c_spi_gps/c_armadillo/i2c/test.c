#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "i2c_scd30.h"

int main(int argc, char **argv)
{
        char *bus = "/dev/i2c-3";

        // Set measurement interval (second)
        scd30_set_value(bus, 0x4600, 2);
        // Set Forced Recalibration value (ppm)
        scd30_set_value(bus, 0x5204, 450);
        // Set Temperature Offset (0.01K)
        scd30_set_value(bus, 0x5403, 200);
        // Activate Automatic Self-Calibration
        scd30_set_value(bus, 0x5306, 1);
        // Altitude Compensation (meter)
        scd30_set_value(bus, 0x5102, 30);
        // Soft Reset
        scd30_reset(bus);

        while (1)
        {
                while (scd30_get_data_ready_status(bus) == 2)
                {
                        //wait for ready
                        usleep(10000);
                }

                float readout[3] = {};

                if (scd30_read_measurement(bus, readout))
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
        return 0;
}