#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "spi_mcp3204.h"

#define LEN 3

int mcp3204_access(char *bus, uint8_t cmd, uint16_t *read_data)
{
        int fd;
        uint8_t tx[LEN] = {};
        uint8_t rx[LEN] = {};

        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)tx,
            .rx_buf = (unsigned long)rx,
            .len = LEN,
            .delay_usecs = 0,
            .speed_hz = 0,
            .bits_per_word = 0
        };

        tx[0] = 0x4 | (cmd >> 2);
        tx[1] = cmd << 6;

        if ((fd = open(bus, O_RDONLY)) < 0)
        {
                fprintf(stderr, "Error: Couldn't open bus\n");
                goto error;
        }

        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0)
        {
                fprintf(stderr, "Error: Unable access to slave\n");
                goto error;
        }

        *read_data = ((rx[1] << 8) | rx[2]) & 0xfff;

        close(fd);
        return 0;

error:
        close(fd);
        return 1;
}

int mcp3204_read_diff(char *bus, ch_diff ch_conf, uint16_t *out)
{
        if (ch_conf < 0 || 3 < ch_conf)
        {
                fprintf(stderr, "Error: Invalid input setting\n");
                return 1;
        }

        if (mcp3204_access(bus, ch_conf, out))
        {
                return 1;
        }
        return 0;
}

int mcp3204_read(char *bus, int ch, uint16_t *out)
{
        if (ch < 0 || 3 < ch)
        {
                fprintf(stderr, "Error: Invalid input setting\n");
                return 1;
        }

        if (mcp3204_access(bus, 0x8 | ch, out))
        {
                return 1;
        }
        return 0;
}
