//
//  raspOLED: 96x64 OLED driver for Raspberry Pi
//  xkozima@myu.ac.jp
//  ref :- https://www.ei.tohoku.ac.jp/xkozima/lab/raspTutorial3.html
//
#include "raspOLED.h"
#include "font6x8.h"
#include <stdlib.h>

//  SPI 縺九ｉ繧ｳ繝槭Φ繝蛾∽ｿ｡�茨ｼ代ヰ繧､繝茨ｼ�
void  OLED::sendCommand1(unsigned char cmdOne)
{
    gpio_clear(OLED_DC_GPIO);
    spi.send1(cmdOne);
}
//  SPI 縺九ｉ繧ｳ繝槭Φ繝蛾∽ｿ｡�郁､�焚繝舌う繝茨ｼ�
void  OLED::sendCommandN(unsigned char *cmd, int length)
{
    gpio_clear(OLED_DC_GPIO);
    spi.sendN(cmd, length);
}
//  SPI 縺九ｉ繝��繧ｿ騾∽ｿ｡�郁､�焚繝舌う繝茨ｼ�
void  OLED::sendDataN(unsigned char *data, int length)
{
    gpio_set(OLED_DC_GPIO);
    spi.sendN(data, length);
}
//  SPI 縺九ｉ繝��繧ｿ騾∽ｿ｡�育判蜒上↑縺ｩ縺ｮ繝舌う繝亥���
void  OLED::sendBuffer(unsigned char *buffer, int length)
{
    gpio_set(OLED_DC_GPIO);
    spi.sendBuffer(buffer, length);
}
 
//  OLED 蛻晄悄蛹厄ｼ�"/dev/spidev0.0", 8000000);
void  OLED::init ()
{
    init("/dev/spidev0.0", 8000000);
}
void  OLED::init (const char *spiDevice)
{
    init(spiDevice, 8000000);
}
void  OLED::init (const char *spiDevice, int clock)
{
    //  setup gpio/spi (buffer 8192byte; 16MHz)
    gpio_init();
    spi.init(spiDevice, clock);
    //  setup rst/cs/dc pins
    gpio_configure(OLED_DC_GPIO,  GPIO_OUTPUT);
    gpio_configure(OLED_RST_GPIO, GPIO_OUTPUT);
    //  clear cs/dc/rst (low active)
    gpio_set(OLED_DC_GPIO);
    gpio_set(OLED_RST_GPIO);
    //  reset OLED (low active)
    gpio_clear(OLED_RST_GPIO);
    usleep(20000);
    gpio_set(OLED_RST_GPIO);
    usleep(20000);
    //  init command
    unsigned char cmd1[] = {0xa0, 0x70};
    sendCommandN(cmd1, 2);
    unsigned char cmd2[] = {0xaf};
    sendCommandN(cmd2, 1);
    unsigned char cmd3[] = {0x26, 0x01};
    sendCommandN(cmd3, 2);
    //  clear flags
    reversal = false;
    filling = true;
    //  clear and display on
    rect(0, 0, 96, 64, 0, 1);
    usleep(100000);
    sendCommand1(0xaf);    //  display on
}

//  16bit(RGB565) 濶ｲ諠��ｱ繧� 24bit 濶ｲ諠��ｱ縺九ｉ逕滓�
unsigned short OLED::color (int r, int g, int b)
{
    return ((b & 0xf8) << 8) | ((g & 0xfc) << 3) | ((r & 0xf8) >> 3);
}

//  逕ｻ髱｢繧ｯ繝ｪ繧｢��0: 鮟抵ｼ�0xffff: 逋ｽ, etc.��
void OLED::clear (unsigned short color)
{
    rect(0, 0, 96, 64, color, 1);
}

//  轤ｹ繧呈緒逕ｻ
void OLED::point(int x, int y, unsigned short color)
{
    //  draw a point (a small rectangle/filled)
    unsigned char c1 = (color & 0xf800) >> 10, 
                  c2 = (color & 0x07e0) >> 5, 
                  c3 = (color & 0x001f) << 1;
    unsigned char cmd[] = {0x22, x, y, x, y,  
                           c1, c2, c3,  c1, c2, c3 };
    sendCommandN(cmd, 11);
}

//  髟ｷ譁ｹ蠖｢繧呈緒逕ｻ��(fill)? 蝪励ｊ縺､縺ｶ縺�: 霈ｪ驛ｭ縺ｮ縺ｿ��
void OLED::rect (int x, int y, int w, int h,
                 unsigned short color, int fill )
{
    unsigned char c1 = (color & 0xf800) >> 10, 
                  c2 = (color & 0x07e0) >> 5, 
                  c3 = (color & 0x001f) << 1;
    //  
    //  fill or no fill
    if (fill) {
        if (! filling) {
            unsigned char cmdF[] = {0x26, 1};
            sendCommandN(cmdF, 2); 
            filling = true;     
        }
    }
    else {
        if (filling) {
            unsigned char cmdN[] = {0x26, 0};
            sendCommandN(cmdN, 2);
            filling = false;
        }
    }
    //  un-reverse
    if (reversal) {
        unsigned char cmdN[] = {0xa0, 0x70};
        sendCommandN(cmdN, 2);
        reversal = false;
    }
    //  draw/fill the rectangle
    unsigned char cmd[] = {0x22, x, y, x + w - 1, y + h - 1, 
                           c1, c2, c3,  c1, c2, c3 };
    sendCommandN(cmd, 11);
    //  wait
    usleep(w * h / 10);
}

//  邱壹ｒ謠冗判��SSD1332 縺ｮ謠冗判繝舌げ縺ｫ蟇ｾ蠢懶ｼ�
void OLED::line(int x1, int y1, int x2, int y2, unsigned short color)
{
    unsigned char c1 = (color & 0xf800) >> 10, 
                  c2 = (color & 0x07e0) >> 5, 
                  c3 = (color & 0x001f) << 1;
    if (x1 < x2) {
        if (y1 < y2) {
            //  normal
            unsigned char cmd[] = {0x21, x1, y1, x2, y2, c1, c2, c3};
            if (reversal) {
                unsigned char cmdN[] = {0xa0, 0x70};
                sendCommandN(cmdN, 2);
                reversal = false;
            }
            sendCommandN(cmd, 8);
        }
        else if (y1 > y2) {
            //  reversal
            if (! reversal) {
                unsigned char cmdR[] = {0xa0, 0x72};
                sendCommandN(cmdR, 2);
                reversal = true;
            }
            unsigned char cmd[]  = {0x21, 95 - x2, y2, 95 - x1, y1, c1, c2, c3};
            sendCommandN(cmd, 8);
        }
        else {  //  y1 == y2
            //  normal
            unsigned char cmd[] = {0x21, x1, y1, x2, y2, c1, c2, c3};
            if (reversal) {
                unsigned char cmdN[] = {0xa0, 0x70};
                sendCommandN(cmdN, 2);
                reversal = false;
            }
            sendCommandN(cmd, 8);
        }
    }
    else if (x1 > x2) {
        if (y1 < y2) {
            //  reversal
            if (! reversal) {
                unsigned char cmdR[] = {0xa0, 0x72};
                sendCommandN(cmdR, 2);
                reversal = true;
            }
            unsigned char cmd[]  = {0x21, 95 - x2, y2, 95 - x1, y1, c1, c2, c3};
            sendCommandN(cmd, 8);
        }
        else if (y1 > y2) {
            //  normal
            unsigned char cmd[] = {0x21, x1, y1, x2, y2, c1, c2, c3};
            if (reversal) {
                unsigned char cmdN[] = {0xa0, 0x70};
                sendCommandN(cmdN, 2);
                reversal = false;
            }
            sendCommandN(cmd, 8);
        }
        else {  //  y1 == y2
            //  reversal
            unsigned char cmd[] = {0xa0, 0x72, 
                                   0x21, 95 - x2, y2, 95 - x1, y1, c1, c2, c3,
                                   0xa0, 0x70 };
            sendCommandN(cmd, 12);
        }
    }
    else {  //  x1 == x2
        if (y1 < y2) {
            //  normal
            unsigned char cmd[] = {0x21, x1, y1, x2, y2, c1, c2, c3};
            if (reversal) {
                unsigned char cmdN[] = {0xa0, 0x70};
                sendCommandN(cmdN, 2);
                reversal = false;
            }
            sendCommandN(cmd, 8);
        }
        else if (y1 > y2) {
            //  normal (force downward)
            unsigned char cmd[]  = {0x21, x2, y2, x1, y1, c1, c2, c3};
            sendCommandN(cmd, 8);
        }
        else {  //  y1 == y2
            //  normal (point)
            unsigned char cmd[] = {0x21, x1, y1, x2, y2, c1, c2, c3};
            if (reversal) {
                unsigned char cmdN[] = {0xa0, 0x70};
                sendCommandN(cmdN, 2);
                reversal = false;
            }
            sendCommandN(cmd, 8);
        }
    }
    //  wait
    usleep(1);
}
 
//  蜀�ｒ謠冗判��(fill)? 蝪励ｊ縺､縺ｶ縺�: 霈ｪ驛ｭ縺ｮ縺ｿ��
void OLED::circle(int x, int y, int r, unsigned short color, int fill)
{
    //  exception
    if (r < 1) {
        line(x, y, x, y, color);
        return;
    } else if (r == 1) {
        rect(x - 1, y - 1, 3, 3, color, fill);
        return;
    }
    //  ok, go
    int xx = 0, 
        yy = r, 
        err = 2 - 2 * r;
    //  iteration
    do {
        //  draw it! 
        if (fill) {
            line(x + xx, y + yy, x + xx, y, color);
            line(x - xx, y + yy, x, y + yy, color);
            line(x - xx, y - yy, x - xx, y, color);
            line(x + xx, y - yy, x, y - yy, color);
        }
        else {
            point(x + xx, y + yy, color);
            point(x - xx, y + yy, color);
            point(x - xx, y - yy, color);
            point(x + xx, y - yy, color);
        }
        //  walk
        if (err > -yy) {
            yy--;
            err += 1 - 2 * yy;
        }
        if (err <= xx) {
            xx++;
            err += 1 + 2 * xx;
        }
    } while (yy >= 0);
}

//  讌募�繧呈緒逕ｻ��rx/ry: 蜊雁ｾ�ｼ�(fill)? 蝪励ｊ縺､縺ｶ縺�: 霈ｪ驛ｭ縺ｮ縺ｿ�� 
void OLED::ellipse(int x, int y, int rx, int ry, unsigned short color, int fill)
{
    //  exception
    if (rx < 1) {
        line(x, y - ry, x, y + ry, color);
        return;
    }
    else if (ry < 1) {
        line(x - rx, y, x + rx, y, color);
        return;
    }
    //  ok, go
    int xx = -rx, yy = 0;
    int e2 = ry, 
        dx = (2 * xx + 1) * e2 * e2, 
        dy = xx * xx, 
        err = dx + dy;
    do {
        if (fill) {
            line(x + xx, y + yy, x + xx, y, color);
            line(x - xx, y + yy, x, y + yy, color);
            line(x - xx, y - yy, x - xx, y, color);
            line(x + xx, y - yy, x, y - yy, color);
        }
        else {
            point(x + xx, y + yy, color);
            point(x - xx, y + yy, color);
            point(x - xx, y - yy, color);
            point(x + xx, y - yy, color);
        }
        e2 = 2 * err;
        if (e2 >= dx) {
            xx++;
            dx += 2 * ry * ry;
            err += dx;
        }
        if (e2 <= dy) {
            yy++;
            dy += 2 * rx * rx;
            err += dy;
        }
    } while (xx <= 0);
    if (fill) {
        while (yy++ < ry) {
            line(x, y + yy, x, y, color);
            line(x, y - yy, x, y, color);
        }
    }
    else {
        while (yy++ < ry) {
            point(x, y + yy, color);
            point(x, y - yy, color);
        }
    }
}
 
//  逕ｻ蜒上ｒ謠冗判��16bit(RGB565) or 24bit��
void OLED::image(int x, int y, int w, int h, unsigned short *image16)
{
    //  un-reverse
    if (reversal) {
        unsigned char cmdN[] = {0xa0, 0x70};
        sendCommandN(cmdN, 2);
        reversal = false;
    }
    //  setup the region to fill
    unsigned char cmd[]={0x15, x, x+w-1, 0x75, y, y+h-1};
    sendCommandN(cmd, 6);
    //  output through spi (96x64x2 bytes)
    sendBuffer((unsigned char *) image16, 96 * 64 * 2);
    //  wait
    usleep(100);
}
void OLED::image(int x, int y, int w, int h, const unsigned short *image16)
{
    image(x, y, w, h, (unsigned short *) image16);
}
void OLED::image(int x, int y, int w, int h, unsigned char *image24)
{
    //  un-reverse
    if (reversal) {
        unsigned char cmdN[] = {0xa0, 0x70};
        sendCommandN(cmdN, 2);
        reversal = false;
    }
    //  setup the region to fill
    unsigned char cmd[]={0x15, x, x+w-1, 0x75, y, y+h-1};
    sendCommandN(cmd, 6);
    //  fill up the region with the image data
    int size = w * h;
    int bytes = size * sizeof(short);
    unsigned char *data = (unsigned char *) malloc(bytes);
    for (int k = 0; k < size; k++) {
        unsigned char r, g, b;
        b = *image24++;
        g = *image24++;
        r = *image24++;
        unsigned short col = color(r, g, b);
        data[k * 2]     = col >> 8;
        data[k * 2 + 1] = col & 0xff;
    }
    //  output through spi (96x64x2 bytes)
    sendBuffer(data, bytes);
    //  wait and release
    usleep(100);
    free(data);
}
void OLED::image(int x, int y, int w, int h, const unsigned char *image8x3)
{
    image(x, y, w, h, (unsigned char *) image8x3);
}

//  ASCII 譁�ｭ怜�繧呈緒逕ｻ
void OLED::text(int x, int y, char *string, unsigned short color)
{
    int len = strlen(string);
    for (int i = 0; i < len; i++) {
        int xx = x + i * 6;
        unsigned char code = string[i];
        if (code < 0x20 || code >= 0x80) continue;
        unsigned char *font = font6x8 + ((code - 0x20) * 6);
        for (int fx = 0; fx < 6; fx++) {
            if (xx + fx > 95) break;
            unsigned char line = *(font + fx);
            for (int fy = 0, bit = 0x01; fy < 8; fy++, bit <<= 1) {
                if (line & bit) point(xx + fx, y + fy, color);
            }
        }
    }
}
 
//  ASCII 譁�ｭ怜�繧呈緒逕ｻ�郁レ譎ｯ濶ｲ繧よ欠螳夲ｼ�
void OLED::text(int x, int y, char *string, unsigned short colorF, unsigned short colorB)
{
    //  clear the background
    int w = strlen(string) * 6;
    if (x + w > 95) w = 95 - x;
    rect(x, y, w, 8, colorB, 1);
    //  overwrite the text
    text(x, y, string, colorF);
}

//