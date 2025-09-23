#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "spi_mcp3204_misra.h"

#define LEN 3

int mcp3204_access_misra(char *bus, uint8_t cmd, uint16_t *read_data)
{
        int fd;
        uint8_t tx[LEN] = {};
        uint8_t rx[LEN] = {};
		int ret_v = 0;
		uint8_t file_opened = 0;
		
        struct spi_ioc_transfer tr = {
            .tx_buf = (uint64_t)tx,
            .rx_buf = (uint64_t)rx,
            .len = LEN,
            .delay_usecs = 0,
            .speed_hz = 0,
            .bits_per_word = 0
        };
			
        if ((*bus == NULL) || (*read_data == NULL))
		{
			ret_v = 3;
		}
		else
		{
            tx[0] = 0x4 | (cmd >> 2);
            tx[1] = cmd << 6;

            if ((fd = open(bus, O_RDONLY)) < 0)
            {
                fprintf(stderr, "Error: Couldn't open bus\n");
                ret_v = 1;
            }
            else
			{
				file_opened = 1;
                if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0)
                {
                    fprintf(stderr, "Error: Unable access to slave\n");
                    ret_v = 1;
                }
                *read_data = ((rx[1] << 8) | rx[2]) & 0x0fffu;
			}
			if (file_opened == 1)
			{
				close(fd);
			}
        }
		
        return ret_v;

}

int mcp3204_read_diff_misra(char *bus, ch_diff ch_conf, uint16_t *out)
{
	    int ret_v = 0;
		
		if (*bus == NULL)
		{
			ret_v = 3;
		}
		else
		{
            if (ch_conf < 0 || 3 < ch_conf)
            {
                fprintf(stderr, "Error: Invalid input setting\n");
                ret_v = 1;
            }
            else
			{
                if (mcp3204_access(bus, ch_conf, out))
                {
                    ret_v = 1;
                }
			}
		}
        return ret_v;
}

int mcp3204_read_misra(char *bus, int ch, uint16_t *out)
{
	    int ret_v = 0;

		if (*bus == NULL)
		{
			ret_v = 3;
		}
		else
		{		
            if (ch < 0 || 3 < ch)
            {
                fprintf(stderr, "Error: Invalid input setting\n");
                ret_v = 1;
            }
            else
			{
                if (mcp3204_access(bus, 0x8 | ch, out))
                {
                    ret_v = 1;
                }
			}
		}
        return ret_v;
}
