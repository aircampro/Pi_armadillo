//
//  raspSPI: SPI (Serial Peripheral Interface) module
//      xkozima@myu.ac.jp
#include "raspSPI.h"

#define SPI_BLOCKSIZE 2048              //  繝悶Ο繝�け霆｢騾√し繧､繧ｺ�育判蜒冗ｭ会ｼ�
#define SPI_DELAY     0

//  SPI 繝｢繧ｸ繝･繝ｼ繝ｫ縺ｮ蛻晄悄蛹厄ｼ域怙蛻昴↓�大屓縺�縺大他縺ｳ蜃ｺ縺�: "/dev/spidev0.0", 8MHz��
void SPI::init ()
{
    init("/dev/spidev0.0", 8000000);
}
//  繝槭ル繝･繧｢繝ｫ蛻晄悄蛹�
//  ex) spi_init("/dev/spidev0.1", 1000000);  //  1MHz clock
void SPI::init (const char *device, int clockInHz)
{
    int  ret;
    if (device == NULL) device = SPI_DEVICE;
    if (clockInHz == 0) clock = SPI_SPEED; else clock = clockInHz;
    //  SPI 繝�ヰ繧､繧ｹ繧帝幕縺�
    fd = open(device, O_RDWR);
    if (fd < 0) {
        printf("error: cannot open %s (SPI::init)\n", SPI_DEVICE);
        exit(-1);
    }
    //  霆｢騾√Δ繝ｼ繝� 0 繧帝∽ｿ｡繝ｻ蜿嶺ｿ｡縺ｫ險ｭ螳夲ｼ医が繝槭ず繝翫う��
    unsigned char mode = SPI_MODE_0;
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret < 0) {
        printf("error: cannot set WR_MODE to %d (SPI::init)\n", mode);
        close(fd);
        exit(-1);
    }
    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
    if (ret < 0) {
        printf("error: cannot set RD_MODE to %d (SPI::init)\n", mode);
        close(fd);
        exit(-1);
    }
    //  繝薙ャ繝域焚繧帝∽ｿ｡繝ｻ蜿嶺ｿ｡縺ｫ險ｭ螳夲ｼ�8bit 縺励°蜿励￠莉倥￠縺ｪ縺�ｼ�
    unsigned char bits = SPI_BITS;
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0) {
        printf("error: cannot set WR_BITS_PER_WORD to %d (SPI::init)\n", bits);
        close(fd);
        exit(-1);
    }
    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret < 0) {
        printf("error: cannot set RD_BITS_PER_WORD to %d (SPI::init)\n", bits);
        close(fd);
        exit(-1);
    }
}

//  SPI 繧剃ｽｿ縺�ｵゅｏ縺｣縺溘ｉ蜻ｼ縺ｳ蜃ｺ縺呻ｼ育､ｼ蜆豁｣縺励＞莠ｺ縺ｮ縺ｿ��
void SPI::quit ()
{
    close(fd);
    free(trStruct);
}

//  �代ヰ繧､繝医ョ繝ｼ繧ｿ縺ｮ蜊倡匱騾∽ｿ｡
void SPI::send1 (unsigned char data)
{
    //  setup a block
    static unsigned char tdata[1];
    tdata[0] = data;
    struct spi_ioc_transfer tr[1];
    tr[0].tx_buf = (unsigned int) tdata;
    tr[0].rx_buf = (unsigned int) NULL;
    tr[0].len    = 1;
    tr[0].speed_hz      = clock;
    tr[0].delay_usecs   = SPI_DELAY;
    tr[0].bits_per_word = SPI_BITS;
    tr[0].cs_change     = 0;
    //  send this byte
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), tr);
    if (ret < 0) {
        printf("error: cannot send spi message (SPI::send1)\n");
        exit(-1);
    }
}
    
//  �ｮ繝舌う繝医ョ繝ｼ繧ｿ縺ｮ騾∽ｿ｡��N <= SPI_BUFFERSIZE��
void SPI::sendN (unsigned char *data, int n)
{
    //  setup a block
    struct spi_ioc_transfer tr[1];
    tr[0].tx_buf = (unsigned int) data;
    tr[0].rx_buf = (unsigned int) NULL;
    tr[0].len    = n;
    tr[0].speed_hz      = clock;
    tr[0].delay_usecs   = SPI_DELAY;
    tr[0].bits_per_word = SPI_BITS;
    tr[0].cs_change     = 0;
    //  send this byte
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), tr);
    if (ret < 0) {
        printf("error: cannot send spi message (SPI::sendN)\n");
        exit(-1);
    }
}

//  螟ｧ縺阪↑繝舌う繝磯�蛻励�騾∽ｿ｡
void SPI::sendBuffer (unsigned char *buffer, int len)
{
    //  繝悶Ο繝�け謨ｰ縺ｮ險育ｮ�
    int numFullBlocks = len / SPI_BLOCKSIZE;
    int lastBlockSize = len % SPI_BLOCKSIZE;
    //  霆｢騾√ヶ繝ｭ繝�け�域ｺ霈会ｼ峨ｒ騾∽ｿ｡
    int i;
    unsigned char *bufPointer = buffer;
    for (i = 0; i < numFullBlocks; i++) {
        sendN(bufPointer, SPI_BLOCKSIZE);
        bufPointer += SPI_BLOCKSIZE;
    }
    //  遨阪∩谿九＠蛻��霆｢騾√ヶ繝ｭ繝�け繧帝∽ｿ｡�亥ｿ�ｦ√↓蠢懊§縺ｦ��
    if (lastBlockSize) {
        //  遨阪∩谿九＠縺ゅｊ
        sendN(bufPointer, lastBlockSize);
    }
}

//  sendRecN: �ｮ繝舌う繝医ョ繝ｼ繧ｿ縺ｮ騾∽ｿ｡縺ｨ蜿嶺ｿ｡��2048繝舌う繝医∪縺ｧ��
void SPI::sendRecN (unsigned char *send, unsigned char *rec, int n)
{
    //  setup a block
    struct spi_ioc_transfer tr[1];
    tr[0].tx_buf = (unsigned int) send;
    tr[0].rx_buf = (unsigned int) rec;
    tr[0].len    = n;
    tr[0].speed_hz      = clock;
    tr[0].delay_usecs   = SPI_DELAY;
    tr[0].bits_per_word = SPI_BITS;
    tr[0].cs_change     = 0;
    //  send this byte
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), tr);
    if (ret < 0) {
        printf("error: cannot send spi message (SPI::sendRecN)\n");
        exit(-1);
    }
}

//