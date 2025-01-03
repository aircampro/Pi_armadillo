// reads a gas sensor connected to the LTC2450 ADC on spi
//
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <iostream>                        
#include <cmath>

#define PPM_HIGH 0.9f

// sensore ref:- # https://www.az-delivery.uk/products/mq-135-gas-sensor-modul
//
double mq135_gas_ppm(double analog_v, double ref_v=3.3 double raw_c=4095.0, double RL = 10.0, double Ro = 0.54, double m = -0.280, double b = 0.425) {
	
    sensor_volt = analog_v * (ref_v / raw_c);  
    RS_gas = (RL * (ref_v - sensor_volt) / sensor_volt);
    RS_ratio = RS_gas / Ro;
    ppm = log10(RS_ratio) * m + b;                                        // Calculate PPM (use the correct logarithmic formula)
    return ppm
}

int main() {
	
    int fd;
    uint16_t spiMode     = SPI_MODE_0;
    uint8_t  bitsPerWord = 8u;
    uint32_t spiSpeed    = 125.0f / 2.0f * 1000.0f * 1000.0f;

    fd = open("/dev/spidev0.0", O_RDWR);

    ioctl(fd, SPI_IOC_WR_MODE, &spiMode);
    ioctl(fd, SPI_IOC_RD_MODE, &spiMode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord);
    ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bitsPerWord);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spiSpeed);
    ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &spiSpeed);

    /* LTC2450 ADC on SPI */
    uint8_t tx[2] = {0x00u, 0x00u};
    uint8_t rx[2];
    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (uint64_t)tx;
    tr.rx_buf = (uint64_t)rx;
    tr.len = 2;
    tr.speed_hz = spiSpeed;
    tr.delay_usecs = 0;
    tr.bits_per_word = bitsPerWord;
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    uint16_t dataV = (tr.rx_buf[0] & 0xFFu) << 8u | (tr.rx_buf[1] & 0xFFu); 
    double ppm = mq135_gas_ppm( static_cast<double>(dataV) );
    printf(" the gas ppm %f\n", ppm);

    // if over set-point write 1 to GPIO 26 else reset it to 0
    int fd2;
    fd2 = open("/sys/class/gpio/export", O_WRONLY);
    write(fd2, "26", 2);
    close(fd2);

    fd2 = open("/sys/class/gpio/gpio26/direction", O_WRONLY);
    write(fd2, "out", 3);
    close(fd2);

    fd2 = open("/sys/class/gpio/gpio26/value", O_WRONLY);
	if (ppm > PPM_HIGH) { 
        write(fd2, "1", 1);
    } else {
        write(fd2, "0", 1);
    }
    close(fd2);
	
    close(fd);
    return(0);	
}