#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "spi_mcp3204.h"

int main(int argc, char **argv)
{
        char *bus = "/dev/spidev0.0";
        int ret_val = 0;
		
        while (1)
        {
                uint16_t readout = 0;

                for (int ch = 0; ch < 4; ch++)
                {
                        if (mcp3204_read_misra(bus, ch, &readout))
                        {
                                fprintf(stderr, "Error mcp3204_read\n");
								ret_val = 1;
                        }
                        else
                        {
                                printf("ch%d: %4d  ", ch, readout);
                        }
                }

                if (mcp3204_read_diff_misra(bus, P0_N1, &readout))
                {
                        fprintf(stderr, "Error mcp3204_read_diff\n");
						ret_val = 1;
                }
                else
                {
                        printf("diff(ch0-ch1): %4d\n", readout);
                }
                
                sleep(1);
        }
        return ret_val;
}