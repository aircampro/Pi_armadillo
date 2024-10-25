//
//  96x64 OLED driver for Raspberry Pi
//      xkozima@myu.ac.jp
//      ref :- https://www.ei.tohoku.ac.jp/xkozima/lab/raspTutorial3.html
//
#ifndef OLED_H
#define OLED_H

#include <string.h>
#include "raspGPIO.h"
#include "raspSPI.h"
#define OLED_DC_GPIO   17               //  RPi 縺九ｉ DC 縺ｸ縺ｮ GPIO 繝斐Φ
#define OLED_RST_GPIO  22               //  RPi 縺九ｉ RST 縺ｸ縺ｮ GPIO 繝斐Φ

class OLED {
public:
    //  OLED 蛻晄悄蛹厄ｼ�"/dev/spidev0.0", 8000000);
    void init();
    void init(const char *spiDevice);
    void init(const char *spiDevice, int clock);
    //  逕ｻ髱｢繧ｯ繝ｪ繧｢��0: 鮟抵ｼ�0xffff: 逋ｽ, etc.��
    void clear(unsigned short color);
    //  轤ｹ繧呈緒逕ｻ
    void point(int x, int y, unsigned short color);
    //  邱壹ｒ謠冗判
    void line(int x1, int y1, int x2, int y2, unsigned short color);
    //  髟ｷ譁ｹ蠖｢繧呈緒逕ｻ��(fill)? 蝪励ｊ縺､縺ｶ縺�: 霈ｪ驛ｭ縺ｮ縺ｿ��
    void rect(int x, int y, int w, int h, unsigned short color, int fill);
    //  蜀�ｒ謠冗判��(fill)? 蝪励ｊ縺､縺ｶ縺�: 霈ｪ驛ｭ縺ｮ縺ｿ��
    void circle(int x, int y, int r, unsigned short color, int fill);
    //  讌募�繧呈緒逕ｻ��rx/ry: 蜊雁ｾ�ｼ�(fill)? 蝪励ｊ縺､縺ｶ縺�: 霈ｪ驛ｭ縺ｮ縺ｿ��
    void ellipse(int x, int y, int rx, int ry, unsigned short color, int fill);
    //  逕ｻ蜒上ｒ謠冗判��16bit(RGB565) or 24bit��
    void image(int x, int y, int w, int h, unsigned short *image16);
    void image(int x, int y, int w, int h, const unsigned short *image16);
    void image(int x, int y, int w, int h, unsigned char *image24);
    void image(int x, int y, int w, int h, const unsigned char *image24);
    //  ASCII 譁�ｭ怜�繧呈緒逕ｻ
    void text(int x, int y, char *string, unsigned short color);
    //  ASCII 譁�ｭ怜�繧呈緒逕ｻ�郁レ譎ｯ濶ｲ繧よ欠螳夲ｼ�
    void text(int x, int y, char *string, unsigned short colorF, unsigned short colorB);
    //  16bit(RGB565) 濶ｲ諠��ｱ繧� 24bit 濶ｲ諠��ｱ縺九ｉ逕滓�
    unsigned short color(int r, int g, int b);
private:
    SPI spi;
    void sendCommand1(unsigned char cmdOne);
    void sendCommandN(unsigned char *cmd, int length);
    void sendDataN(unsigned char *data, int length);
    void sendBuffer(unsigned char *buffer, int length);
    bool reversal;
    bool filling;
};

#endif