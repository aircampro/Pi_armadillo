//
//  raspADC: Raspberry Pi ADC (Analog-Digital Convertor) module
//      xkozima@myu.ac.jp
//      (for MCP3204/08 SPI)
#include "raspADC.h"

//  clock/mosi/miso 縺ｮ霆｢騾√ち繧､繝溘Φ繧ｰ
//  00 01 02 03 04 05 06 07   08 09 10 11 12 13 14 15  16 17 18 19 20 21 22 23
//   0  0  0  0  0  1  1 d2   d1 d0  X  x  x  x  x  x   x  x  x  x  x  x  x  x
//   -  -  -  -  -  -  -  -    -  -  -  0 aC aB aA a8  a7 a6 a5 a4 a3 a2 a1 a0

//  ADC 繝｢繧ｸ繝･繝ｼ繝ｫ縺ｮ蛻晄悄蛹�
//      繝�ヵ繧ｩ繝ｫ繝亥､: init("/dev/spidev0.1", ADC_3208);
void ADC::init()
{
    init("/dev/spidev0.1", ADC_3208);
}
void ADC::init(const char *spiDevice)
{
    init(spiDevice, ADC_3208);
}
void ADC::init(int adc320X)
{
    init("/dev/spidev0.1", adc320X);
}
void ADC::init(const char *spiDevice, int adc320X)
{
    spi.init(spiDevice, ADC_CLOCK);
    adcChip = adc320X;
}
void ADC::quit()
{
    spi.quit();
}

int ADC::get(int channel)
{
    //  ADC_3204 or ADC_3208
    //  SPI 3byte (4ch/8ch; 12bit data = 0..4095)
    unsigned char send[3], rec[3];
    int ret = -1;
    send[0] = (channel & 0x04)? 0x07: 0x06;
    send[1] = (channel & 0x03) << 6;
    send[2] = 0;
    //  send and receive
	//  uncomment not to try to re-connect after error
    //  spi.sendRecN(send, rec, 3);
	if ((spi.sendRecNRetrys(send, rec, 3) == -1) {              // was error
       usleep(10000);
	   init(ADC_SPI, ADC_3208);
       usleep(10000);
	   if ((spi.sendRecNRetrys(send, rec, 3) != -1) {           // now no error
          ret = ((rec[1] & 0x0f) << 8) | rec[2];		   
       }
	} else {
      ret = ((rec[1] & 0x0f) << 8) | rec[2];
    }	  
    //  return the 12bit result
    return ret;
}

//