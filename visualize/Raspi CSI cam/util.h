#ifndef UTIL_H_
#define UTIL_H_
// common utilities for video codecs and conversions 
//
#include <common.h>
#include "common.h"

#define SWAP16BIT(data) (((data&0xFF) << 8) | ((data >> 8) & 0xFF))

RET saveFileBinary(const char* filename, uint8_t* data, int size);
void convertRGB888To565(uint8_t *src, uint8_t *dst, uint32_t pixelSize);
void convertYUYVToRGB565(uint8_t *src, uint8_t *dst, uint32_t pixelSize);
void convertYVYUToRGB565(uint8_t *src, uint8_t *dst, uint32_t pixelSize);
void convert_xyz_lab( float *X, float *Y, float *Z, uint16_t *L, uint16_t *A, uint16_t *B);
void convert_cmyk_rgb( uint16_t C, uint16_t M, uint16_t Y, uint16_t K, uint16_t *R, uint16_t *G, uint16_t *B );
void NV12ToRGB(uint8_t * rgbBuffer, uint8_t * yuvBuffer, int16_t width, int16_t height);
int8_t convert_YUV_420_422( uint8_t *Cin, uint8_t *Cout, uint16_t Clen);
void NV12_YUV420P(const unsigned char* image_src, unsigned char* image_dst, int16_t image_width, int16_t image_height);
void RGB2HSL(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *h, uint8_t *s, uint8_t *l );
void average_image(int16_t w, int16_t h, double *img, double *img_avg);
uint16_t abToHue(float a, float b);
void labToLch( float a, float b, uint16_t *c, uint16_t *h );
uint16_t GetSmallestValue(uint16_t x, uint16_t y, uint16_t z );
void convert_rgb_cmyk( uint16_t R, uint16_t G, uint16_t B, uint16_t *C, uint16_t *M, uint16_t *Y, uint16_t *K);
void resize_uyuv(uint8_t *source, uint8_t *dest, int16_t downsample, int input_w, int output_h, int output_w);
void grayscale_uyvy(uint8_t *source, uint8_t *dest, int output_h, int output_w);
RET colorfilt_uyvy(uint8_t *source, uint8_t *dest, int output_h, int output_w, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M);
uint8_t convertRGBtoRGB332byte(uint8_t r, uint8_t g, uint8_t b);
uint8_t convertRGB888toRGB332byte( uint8_t r, uint8_t g, uint8_t b);
uint8_t convertRGB888toRGB565byte( uint8_t r, uint8_t g, uint8_t b);
void convertRGB888To332(uint8_t *src, uint8_t *dst, uint32_t pixelSize);
void convertRGB332To888(uint8_t *src, uint8_t *dst, uint32_t pixelSize);
#endif
