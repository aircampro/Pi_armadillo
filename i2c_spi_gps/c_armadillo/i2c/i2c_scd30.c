#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

const uint8_t table_crc8_maxim[] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC};

uint8_t crc_calc(uint8_t *in)
{
        uint8_t data = 0xff;
        data = table_crc8_maxim[data ^ *in];
        data = table_crc8_maxim[data ^ *(in + 1)];

        return data;
}

int crc_check(uint8_t *in)
{
        return (crc_calc(in) != *(in + 2));
}

int scd30_access(char *bus, uint16_t cmd, int write_data_count, uint16_t write_data, int read_data_count, uint16_t *read_data)
{
        // write_data_count = 0 or 1
        // read_data_count = 0 to 6

        int fd;
        int write_data_count_buff = 2 + (write_data_count == 1) * 3;
        int read_data_count_buff = read_data_count * 3;
        uint8_t write_data_buf[write_data_count_buff];
        uint8_t read_data_buf[read_data_count_buff];

        write_data_buf[0] = cmd >> 8;
        write_data_buf[1] = cmd;
        if (write_data_count == 1)
        {
                write_data_buf[2] = write_data >> 8;
                write_data_buf[3] = write_data;
                write_data_buf[4] = crc_calc(&write_data_buf[2]);
        }

        if ((fd = open(bus, O_RDWR)) < 0)
        {
                fprintf(stderr, "Error: Couldn't open bus\n");
                goto error;
        }

        if (ioctl(fd, I2C_SLAVE, 0x61) < 0)
        {
                fprintf(stderr, "Error: Unable access to slave\n");
                goto error;
        }

        // write cmd
        if ((write(fd, &write_data_buf, write_data_count_buff)) != write_data_count_buff)
        {
                fprintf(stderr, "Error: Write\n");
                goto error;
        }

        if (read_data_count_buff > 0 && read_data != NULL)
        {
                // read data
                if (read(fd, &read_data_buf, read_data_count_buff) != read_data_count_buff)
                {
                        fprintf(stderr, "Error: Read\n");
                        goto error;
                }

                // check data
                for (int i = 0; i < read_data_count; i++)
                {
                        if (crc_check(read_data_buf + i * 3))
                        {
                                fprintf(stderr, "Error: CRC-8 Check\n");
                                goto error;
                        }

                        read_data[i] = (uint16_t)read_data_buf[i * 3] << 8 | read_data_buf[i * 3 + 1];
                }
        }

        close(fd);
        return 0;

error:
        close(fd);
        return 1;
}

int scd30_read_measurement(char *bus, float *out)
{
        uint16_t read_data_buf[6];

        if (scd30_access(bus, 0x0300, 0, 0x0000, 6, read_data_buf))
        {
                // error
                return 1;
        }

        if (out != NULL)
        {
                for (int i = 0; i < 3; i++)
                {
                        uint32_t buf = (uint32_t)read_data_buf[i * 2] << 16 | read_data_buf[i * 2 + 1];

                        *(out + i) = *(float *)&buf;
                }
        }

        return 0;
}

int scd30_get_data_ready_status(char *bus)
{
        uint16_t read_data;

        if (scd30_access(bus, 0x0202, 0, 0x0000, 1, &read_data))
        {
                // error
                return 1;
        }

        if (read_data > 1)
        {
                // error
                return 1;
        }

        if (read_data == 0)
        {
                // busy
                return 2;
        }

        // ready
        return 0;
}

int scd30_set_value(char *bus, uint16_t cmd, uint16_t write_data)
{
        if (scd30_access(bus, cmd, 1, write_data, 0, NULL))
        {
                // error
                return 1;
        }

        return 0;
}

int scd30_reset(char *bus)
{
        if (scd30_access(bus, 0xD304, 0, 0x0000, 0, NULL))
        {
                // error
                return 1;
        }

        sleep(3);

        // skip first readout
        scd30_read_measurement(bus, NULL);

        return 0;
}
