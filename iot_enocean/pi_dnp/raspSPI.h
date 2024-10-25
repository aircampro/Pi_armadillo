//
//  Raspberry Pi SPI module
//      xkozima@myu.ac.jp
#ifndef SPI_H
#define SPI_H

#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>

#define SPI_DEVICE    "/dev/spidev0.0"  //  SPI 繝�ヰ繧､繧ｹ�医ョ繝輔か繝ｫ繝茨ｼ�
#define SPI_SPEED     8000000           //  繧ｯ繝ｭ繝�け 8MHz�医ョ繝輔か繝ｫ繝茨ｼ�
#define SPI_BITS      8                 //  繝薙ャ繝域焚��8bit 縺ｮ縺ｿ蜿ｯ閭ｽ��

class SPI {
public:
    //  init: 譛蛻昴↓�大屓縺�縺大他縺ｳ蜃ｺ縺呻ｼ医〒繝舌う繧ｹ蜷阪�繧ｯ繝ｭ繝�け蜻ｨ豕｢謨ｰ繧剃ｸ弱∴繧具ｼ�
    void init ();
    void init (const char *device, int clockInHz);
    //  quit: SPI 繧剃ｽｿ縺�ｵゅｏ縺｣縺溘ｉ蜻ｼ縺ｳ蜃ｺ縺呻ｼ育､ｼ蜆豁｣縺励＞莠ｺ縺ｮ縺ｿ��
    void quit ();
    //  sendBuffer: 螟ｧ縺阪↑繝舌う繝磯�蛻励�騾∽ｿ｡
    void sendBuffer (unsigned char *data, int len);
    //  sendN: �ｮ繝舌う繝医ョ繝ｼ繧ｿ縺ｮ騾∽ｿ｡��n <= 2048��
    void sendN (unsigned char *data, int n);
    //  send1: �代ヰ繧､繝医ョ繝ｼ繧ｿ縺ｮ蜊倡匱騾∽ｿ｡
    void send1 (unsigned char data);
    //  receiveN: �ｮ繝舌う繝医ョ繝ｼ繧ｿ縺ｮ蜿嶺ｿ｡
    void sendRecN (unsigned char *send, unsigned char *rec, int n);
private:
    //  繝輔ぃ繧､繝ｫ繝�ぅ繧ｹ繧ｯ繝ｪ繝励ち / 繧ｯ繝ｭ繝�け(Hz)
    int fd;
    int clock;
    //  貅霈峨�繝悶Ο繝�け謨ｰ / 遨阪∩谿九＠繝悶Ο繝�け縺ｮ繝舌う繝域焚
    int numFullBlocks, lastBlockSize;
    //  SPI 霆｢騾√�讒矩�菴�
    struct spi_ioc_transfer *trStruct;
};

#endif