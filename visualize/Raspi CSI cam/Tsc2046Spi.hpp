// Tsc2046Spi.h
// Touch Panel (TP)
// TSC2046 SPI interface (ADS7846 compatible)

#ifndef TSC_2046_SPI_H_
#define TSC_2046_SPI_H_

#include <pthread.h>
#include "common.h"

class Tsc2046Spi
{
private:
	const uint32_t INTERRUPT_INTERVAL_MS = 50;
	typedef void (*CALLBACK)(float x, float y, int pressure, void* arg);

private:
	/* variables for SPI */
	int m_fdSpi;	                                                     // /dev/spidev0.0
	uint16_t m_spiMode;
	uint8_t m_bitsPerWord;
	uint32_t m_spiSpeed;

	/* variables for GPIO(Irq) */
	char m_gpioPinIrq[4];

	/* variables for thread */
	pthread_t m_thread;
	bool m_isThreadExit;
	CALLBACK m_cbTouch;
	void* m_cbArg;

private:
	static void *threadFunc(void *pInstance);

private:
	RET initializeSPI(const char* spiDevice);
	RET initializeGPIO();
	RET openWriteFile(const char* filePath, const char* str);
	void sendCommand(uint8_t cmd, uint8_t *data0, uint8_t *data1);
	uint8_t createCommand(uint8_t A, uint8_t mode, uint8_t SER);

public:
	Tsc2046Spi(const char* spiDevice, int gpioPinIrq, CALLBACK cb, void *arg);
	~Tsc2046Spi();
	bool getPosition(float *x, float *y, int *temp);	// note: not thread safe

};

#endif /* TSC_2046_SPI_H_ */