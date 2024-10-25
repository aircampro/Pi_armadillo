//
//  raspADC: Raspberry Pi ADC (Analog-Digital Convertor) module
//      xkozima@myu.ac.jp
//      (for MCP3204/08 SPI)
#ifndef ADC_H
#define ADC_H

#include "raspSPI.h"

#define ADC_3204 1
#define ADC_3208 2
#define ADC_CLOCK 1000000           //  1MHz at 2.7V Vcc

class ADC {
public:
    //  ADC 繝｢繧ｸ繝･繝ｼ繝ｫ縺ｮ蛻晄悄蛹�
    void init();
    void init(const char *spiDevice);
    void init(int adc320X);
    void init(const char *spiDevice, int adc320X);
    //  謖�ｮ壹メ繝｣繝阪Ν��0縲�7�峨�繧｢繝翫Ο繧ｰ繝��繧ｿ蜿門ｾ暦ｼ�0縲�4095��
    int get(int channel);
private:
    SPI spi;        //  SPI (raspSPI)
    int adcChip;    //  ADC 縺ｮ遞ｮ鬘橸ｼ�ADC_3204, ADC_3208��
};

#endif