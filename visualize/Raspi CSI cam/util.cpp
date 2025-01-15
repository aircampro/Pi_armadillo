#define _USE_MATH_DEFINES
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <unistd.h>
#include "util.h"

/* ========== format codecs and re-sizers =================================== */
#define CLIP(x) do{if(x < 0){x = 0;} else if(x > 255){x = 255;}} while(0)
#define MAX_BRIGHTNESS 255
/* ----------------- PAL television standard -----------------------------    */
/* YUV to RGB conversions                                                     */
#define CONVERT_YUV_R(Y, V) ((298 * (Y - 16) + 409 * (V - 128) + 128) >> 8)
#define CONVERT_YUV_G(Y, U, V) ((298 * (Y - 16) - 100 * (U - 128) - 208 * (V - 128) + 128) >> 8)
#define CONVERT_YUV_B(Y, U) ((298 * (Y - 16) + 516 * (U - 128) + 128) >> 8)
/* RGB to YUV conversions                                                     */
#define CONVERT_RGB_Yuv(R, G, B) ( (0.299f  * R) + (0.587f * G) + (0.114f * B) )
#define CONVERT_RGB_U(R, G, B) ( (-0.147f  * R) + (-0.289f * G) + (0.436f * B) )
#define CONVERT_RGB_V(R, G, B) ( (0.615f  * R) + (-0.515f * G) + (-0.100f  * B) )
/* ----------------- NTSC television standard -----------------------------   */
/* YIQ to RGB */
#define CONVERT_YIQ_R(Y, I, Q) ( Y + (0.956f * I) + (0.621f * Q) )
#define CONVERT_YIQ_G(Y, I, Q) ( Y - (0.272f * I) - (0.647f * Q) )
#define CONVERT_YIQ_B(Y, I, Q) ( Y - (1.105f * I) + (1.702f * Q) )
/*  RGB to YIQ */
#define CONVERT_RGB_Yiq(R, G, B) ( (0.289f * R) + (0.587f * G) + (0.114f * B) )   /* Note: First line Y = (0.299, 0.587, 0.144) (R,G,B) also gives pure B&W translation for RGB. */
#define CONVERT_RGB_I(R, G, B) ( (0.596f * R) - (0.274f * G) - (0.322f * B) )
#define CONVERT_RGB_Q(R, G, B) ( (0.212f * R) - (0.523f * G) + (0.311f * B) )
/* ----------------- D65 CIE XYZ Zitu --------------------------------------- */
#define CONV_ZITU_X(R,G,B) ( 0.431f*R + 0.342f*G + 0.178f*B )
#define CONV_ZITU_Y(R,G,B) ( 0.222f*R + 0.707f*G + 0.071f*B )
#define CONV_ZITU_Z(R,G,B) ( 0.020f*R + 0.130f*G + 0.939f*B )
#define CONV_ZITU_R(X,Y,Z) ( 3.063f*X -1.393f*Y -0.476f*Z )
#define CONV_ZITU_G(X,Y,Z) ( -0.969f*X + 1.876f*Y + 0.042f*Z )
#define CONV_ZITU_B(X,Y,Z) ( 0.068f*X -0.229f*Y + 1.069f*Z )
/* ---------------- CIE XYZrec601-1 (C illuminant) -------------------------- */
#define CONV_REC601_X(R,G,B) ( 0.607f*R + 0.174f*G + 0.200f*B )
#define CONV_REC601_Y(R,G,B) ( 0.299f*R + 0.587f*G + 0.114f*B )
#define CONV_REC601_Z(G,B) ( 0.066f*G + 1.116f*B )
#define CONV_REC601_R(X,Y,Z) ( 1.910f*X - 0.532f*Y - 0.288f*Z )
#define CONV_REC601_G(X,Y,Z) ( -0.985f*X + 1.999f*Y -0.028f*Z )
#define CONV_REC601_B(X,Y,Z) (  0.058f*X -0.118f*Y + 0.898f*Z )
/* ---------------- CIE XYZccir709 (D65) CIE 1931 XYZ------------------------ */
#define CONV_CCIR_X(R,G,B) ( 0.4124f*R + 0.3576f*G + 0.1805f*B )
#define CONV_CCIR_Y(R,G,B) ( 0.2126f*R + 0.7152f*G + 0.0722f*B )
#define CONV_CCIR_Z(R,G,B) ( 0.0193f*R + 0.1192f*G + 0.9505f*B )
#define CONV_CCIR_R(X,Y,Z) ( 3.241f*X - 1.537f*Y - 0.499f*Z )
#define CONV_CCIR_G(X,Y,Z) ( -0.969f*X + 1.876f*Y + 0.042f*Z )
#define CONV_CCIR_B(X,Y,Z) ( 0.056f*X - 0.204f*Y + 1.057f*Z )
/* The YCC colour space used follows that used by TIFF and JPEG (Rec 601-1) */
#define CONV_YCC_Y(R,G,B)  ( 0.2989f*R + 0.5866f*G + 0.1145f*B )
#define CONV_YCC_Cb(R,G,B) ( -0.1687f*R - 0.3312f*G + 0.5000f*B )
#define CONV_YCC_Cr(R,G,B) ( 0.5000f*R - 0.4183f*G - 0.0816f*B )
#define CONV_YCC_R(Y,Cb,Cr) (Y + 1.4022f*Cr)
#define CONV_YCC_G(Y,Cb,Cr) (Y - 0.3456f*Cb - 0.7145f*Cr)
#define CONV_YCC_B(Y,Cb,Cr) (Y + 1.7710f*Cb)
/* Kodak photo CD */
#define CONV_KODAKYCC_Y(R,G,B)  ( 0.299f*R + 0.587f*G + 0.114f*B )
#define CONV_KODAKYCC_Cb(R,G,B) ( 0.701f*R - 0.587f*G - 0.114f*B )
#define CONV_KODAKYCC_Cr(R,G,B) ( -0.299f*R - 0.587f*G + 0.886f*B )

/*  RGB to CMY */
#define CONVERT_RGB_C(R) (255 - R)
#define CONVERT_RGB_M(G) (255 - G)
#define CONVERT_RGB_Y(B) (255 - B)
/* CMY to RGB */
#define CONVERT_CMY_R(C) (255 - C)
#define CONVERT_CMY_G(M) (255 - M)
#define CONVERT_CMY_B(Y) (255 - Y)
/* RGB to Y BY RY */
#define CONVERT_RGB_Y1(R,G,B) (((0.300f * R) + (0.590f * G)) + (0.110f * B))
#define CONVERT_RGB_BY(R,G,B) (-0.300f * R)-(0.590f * G) + (0.890f * B)
#define CONVERT_RGB_RY(R,G,B) (0.700f * R)-(0.590f * G)- (0.110f * B)
/* Y BY RY to RGB */
#define CONVERT_YBYRY_R(Y,RY) (1.000f * Y) + (1.000f * RY)
#define CONVERT_YBYRY_G(Y,BY,RY) (1.000f * Y) - (0.186f * BY) - (0.508f * RY)
#define CONVERT_YBYRY_B(Y,BY) (1.000f * Y)  + (1.000f * BY)
/* avery lees jfif */
#define CONVERT_JFIF_R(Y,Cr) (Y + (1.402f*(Cr-128)))
#define CONVERT_JFIF_G(Y,Cb,Cr) (Y - (0.34414f * (Cb-128)) - (0.71414f * (Cr-128)))
#define CONVERT_JFIF_B(Y,Cb) (Y + (1.772f * (Cb-128)))
/* for vegetation analysis from RGB values */
#define XS_GREEN_IDX_ExG(R,B) ((2-R)-B)
#define XS_RED_IDX_ExR(R,G) ((1.4f*R)-G)
#define NORMALIZED_DIFF_IDX_NDI(G,R) ((G-R)/(G+R))
#define CIVE(R,G,B) (0.441f*R)-(0.811f*G)+(0.385f*B)+18.78745f
#define XSG_MINUS_XSR(R,G,B) (((2-R)-B)-((1.4f*R)-G))

// for old compilers
double round(double x)
{
  return floor(x + 0.5);
}

/*-----------------------------------------------------------------------------
 *      convert_xyz_lab:  CIE 1931 XYZ colors to CIE L*a*b* 
 *      
 *
 *  Parameters: float *X, float *Y, float *Z, uint16_t *L, uint16_t *A, uint16_t *B
 *  Return:   void
 *----------------------------------------------------------------------------*/
void convert_xyz_lab( float *X, float *Y, float *Z, uint16_t *L, uint16_t *A, uint16_t *B)
{
    *X = *X / 95.047f;
    *Y = *Y / 100.0f;
    *Z = *Z / 108.883f;

    if ( *X > 0.008856f) 
    {
       *X = pow(*X, (1.0f / 3.0f));
    }
    else
    {
       *X = ((*X * 7.787f) + 16.0f) / 116.0f;
    }
    if ( *Y > 0.008856f)
    {
       *Y = pow(*Y, (1.0f / 3.0f));
    }
    else
    {
       *Y = ((*Y * 7.787f) + 16.0f) / 116.0f;
    }
    if ( *Z > 0.008856f)
    {
       *Z = pow(*Z, (1.0f / 3.0f));
    }
    else
    {
       *Z = ((*Z * 7.787f) + 16.0f) / 116.0f;
    }
   *L = static_cast<uint16_t>(round((116.0f * (*Y - 16.0f)),0.0f));
   *A = static_cast<uint16_t>(round((500.0f * (*X - *Y)),0.0f));
   *B = static_cast<uint16_t>(round((200.0f * (*Y - *Z)),0.0f));
}

/*-----------------------------------------------------------------------------
 *      convert_cmyk_rgb:  CMYK to RGB 
 *      
 *
 *  Parameters: uint16_t C, uint16_t M, uint16_t Y, uint16_t K, uint16_t *R, uint16_t *G, uint16_t *B
 *  Return:   void
 *----------------------------------------------------------------------------*/
void convert_cmyk_rgb( uint16_t C, uint16_t M, uint16_t Y, uint16_t K, uint16_t *R, uint16_t *G, uint16_t *B )
{
  uint16_t R1,G1,B1;
  
  R1 = 255U - C - K;
  G1 = 255U - M - K;
  B1 = 255U - Y - K;

  *R = std::max(0U, R1);
  *G = std::max(0U, G1);
  *B = std::max(0U, B1);
}

/*-----------------------------------------------------------------------------
 *      saveFileBinary:  save binary file 
 *      
 *
 *  Parameters: const char* filename, uint8_t* data, int size
 *  Return:   RET
 *----------------------------------------------------------------------------*/
RET saveFileBinary(const char* filename, uint8_t* data, int size)
{
	FILE *fp;
	fp = fopen(filename, "wb");
	if(fp == NULL) {
		LOG_E("fopen\n");
		return RET_ERR;
	}
	fwrite(data, sizeof(uint8_t), size, fp);
	fclose(fp);
	return RET_OK;
}

/*-----------------------------------------------------------------------------
 *      convertRGB888To565:  convert rgb to 565 
 *      
 *
 *  Parameters: uint8_t *src, uint8_t *dst, uint32_t pixelSize
 *  Return:   void
 *----------------------------------------------------------------------------*/
void convertRGB888To565(uint8_t *src, uint8_t *dst, uint32_t pixelSize)
{
	for(uint32_t i = 0; i < pixelSize; i++){
		uint8_t r = (*src++);
		uint8_t g = (*src++);
		uint8_t b = (*src++);
		*dst++ = (r & 0xF8) | ((g >> 5) & 0x07);
		*dst++ = ((g << 3)&0xE0) | ((b >> 3) & 0x1F);
	}
}

/*-----------------------------------------------------------------------------
 *   convertYUYVToRGB565:  convert YUV to 565 
 *      
 *
 *  Parameters: uint8_t *src, uint8_t *dst, uint32_t pixelSize
 *  Return:   void
 *----------------------------------------------------------------------------*/
void convertYUYVToRGB565(uint8_t *src, uint8_t *dst, uint32_t pixelSize)
{
	/* convert YUYV (ITU-R BT.601) to RGB565 */
	for(uint32_t i = 0; i < pixelSize / 2; i++){
		uint8_t y0 = *src++;
		uint8_t u = *src++;
		uint8_t y1 = *src++;
		uint8_t v = *src++;
		uint8_t r0 = 1.164 * (y0 - 16)                   + 1.596 * (v - 128);
		uint8_t g0 = 1.164 * (y0 - 16) - 0.391 * (u-128) - 0.813 * (v - 128);
		uint8_t b0 = 1.164 * (y0 - 16) + 2.018 * (u-128);
		uint8_t r1 = 1.164 * (y1 - 16)                   + 1.596 * (v - 128);
		uint8_t g1 = 1.164 * (y1 - 16) - 0.391 * (u-128) - 0.813 * (v - 128);
		uint8_t b1 = 1.164 * (y1 - 16) + 2.018 * (u-128);
		*dst++ = (r0 & 0xF8) | ((g0 >> 5) & 0x07);
		*dst++ = ((g0 << 3)&0xE0) | ((b0 >> 3) & 0x1F);
		*dst++ = (r1 & 0xF8) | ((g1 >> 5) & 0x07);
		*dst++ = ((g1 << 3)&0xE0) | ((b1 >> 3) & 0x1F);
	}
}

  /*-----------------------------------------------------------------------------
 *      resize_uyuv:  picture resize downsample by rate
 *
 *
 *  Parameters: uint8_t *source, uint8_t *dest, int16_t downsample, int input_w, int output_h, int output_w
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/ 
void resize_uyuv(uint8_t *source, uint8_t *dest, int16_t downsample, int input_w, int output_h, int output_w)
{

  int16_t pixelskip,y,x;
  
  pixelskip = downsample-1;
  for (y=0;y<output_h;y++)
  {
    for (x=0;x<output_w;x+=2)                                                  // YUYV
    {
      *dest++ = *source++;                                                      // U
      *dest++ = *source++;                                                      // Y
      source+=(pixelskip+1)*2;                                                  // now skip 3 pixels
      *dest++ = *source++;                                                      // U
      *dest++ = *source++;                                                      // V
      source+=(pixelskip-1)*2;
    }
    source += pixelskip * input_w * 2;                                         // skip 3 rows
  }
}

void grayscale_uyvy(uint8_t *source, uint8_t *dest, int output_h, int output_w)
{

  int16_t y,x;
  
  source++;
  for (y=0;y<output_h;y++)
  {
    for (x=0;x<output_w;x++)                                                   // UYVY
    {
      *dest++ = 127u;                                                           // U
      *dest++ = *source;                                                        // Y
      source+=2u;
    }
  }
}

RET colorfilt_uyvy(uint8_t *source, uint8_t *dest, int output_h, int output_w, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M)
{
  int16_t cnt = 0;
  int16_t y,x;

  for (y=0;y<output_h;y++)
  {
    for (x=0;x<output_w;x+=2)                                                
    {
        if ((dest[1u] >= y_m)
            && (dest[1u] <= y_M)
            && (dest[0u] >= u_m)
            && (dest[0u] <= u_M)
            && (dest[2u] >= v_m)
            && (dest[2u] <= v_M)  )                                                                      // && (dest[2] > 128))
      {
        cnt=++cnt % INT16_MAX;
        // UYVY
        dest[0u] = 64u;         // U
        dest[1u] = source[1u];  // Y
        dest[2u] = 255u;        // V
        dest[3u] = source[3u];  // Y
      }
      else
      {
        // UYVY
        char u = source[0u]-127;
        u/=4;
        dest[0u] = 127;         // U
        dest[1u] = source[1];   // Y
        u = source[2]-127;
        u/=4;
        dest[2u] = 127;         // V
        dest[3u] = source[3];   // Y
      }
      dest+=4u;
      source+=4u;
    }
  }
  return cnt;
}

/*-----------------------------------------------------------------------------
 *      abToHue:  Converts a and b of Lab color space to Hue of LCH color space. 
 *      
 *
 *  Parameters: float a, float b
 *  Return:   uint16_t
 *----------------------------------------------------------------------------*/
uint16_t abToHue(float a, float b)
{
    uint16_t xBias=0u;
    
    if ((a >= 0.0f) && (b == 0.0f)) {
        return 0u;
    }
    if ((a < 0.0f) && (b == 0.0f)) {
        return 180u;
    }
    if ((a == 0.0f) && (b > 0.0f)) {
        return 90u;
    }
    if ((a == 0.0f) && (b < 0.0f)) {
        return 270u;
    }

    if ((a > 0.0f) && (b > 0.0f)) {
        xBias = 0u;
    } else if (a < 0.0f) {
        xBias = 180u;
    } else if ((a > 0.0f) && (b < 0.0f)) {
        xBias = 360u;
    }
    return ((atan(b / a)) * (180.0f / M_PI)) + xBias;
}

/*-----------------------------------------------------------------------------
 *      labToLch:  Converts Lab color space to Luminance-Chroma-Hue color space. 
 *      
 *
 *  Parameters: float a, float b, uint16_t *c, uint16_t *h
 *  Return:    void
 *----------------------------------------------------------------------------*/
void labToLch( float a, float b, uint16_t *c, uint16_t *h )
{
    *c = round(sqrt((a * a) + (b * b)),0.0f);
    *h = abToHue(a, b);
}

/*-----------------------------------------------------------------------------
 *      GetSmallestValue:  smallest value from three integers 
 *      
 *
 *  Parameters: uint16_t x, uint16_t y, uint16_t z
 *  Return:    uint16_t
 *----------------------------------------------------------------------------*/
uint16_t GetSmallestValue(uint16_t x, uint16_t y, uint16_t z )
{
   if (y < x)
   {
      if (z < y)
        return z;
      else
        return y;
   }
   else if (z < x)
   {
      if (y < z)
        return y;
      else
        return z;
   }
   else
     return x;
}

/*-----------------------------------------------------------------------------
 *      convert_rgb_cmyk:  RGB->CMYK 
 *      
 *
 *  Parameters: uint16_t R, uint16_t G, uint16_t B, uint16_t *C, uint16_t *M, uint16_t *Y, uint16_t *K
 *  Return:    void
 *----------------------------------------------------------------------------*/
void convert_rgb_cmyk( uint16_t R, uint16_t G, uint16_t B, uint16_t *C, uint16_t *M, uint16_t *Y, uint16_t *K)
{
  //uint16_t K;
  *K = GetSmallestValue((255U - R), (255U - G), (255U - B));
  *C = 255U - R - *K;
  *M = 255U - G - *K;
  *Y = 255U - B - *K;
}

/*
Sample Ratio	    format	FOURCC Code	Sort Order	bpp
YUV422	Packed	    YUY2	YUV	                    16bit
YUV420	Planar	    I420	YUV	                    12bit
YUV420	Semi-planar	NV12	YUV	                    12bit
*/
/*-----------------------------------------------------------------------------
 *      NV12ToRGB:  NV12 to RGB 
 *      
 *
 *  Parameters: uint8_t * rgbBuffer, uint8_t * yuvBuffer, int16_t width, int16_t height
 *  Return:    void
 *----------------------------------------------------------------------------*/
void NV12ToRGB(uint8_t * rgbBuffer, uint8_t * yuvBuffer, int16_t width, int16_t height)
{
    uint8_t * uvStart;
    uint8_t y[2U] = { 0U, 0U };
    uint8_t u = 0U;
    uint8_t v = 0U;
    int16_t r = 0U;
    int16_t g = 0U;
    int16_t b = 0U;
    int16_t rowCnt = 0U;
    int16_t colCnt = 0U;
    int16_t cnt = 0U;
    
    uvStart = yuvBuffer + width * height;
    for (rowCnt = 0U; rowCnt < height; rowCnt++)
    {
        for (colCnt = 0U; colCnt < width; colCnt += 2U)
        {
            u = *(uvStart + colCnt + 0U);
            v = *(uvStart + colCnt + 1U);
            for (cnt = 0U; cnt < 2U; cnt++)
            {
                y[cnt] = yuvBuffer[rowCnt * width + colCnt + cnt];
                r = CONVERT_YUV_R(y[cnt], v);
                CLIP(r);
                g = CONVERT_YUV_G(y[cnt], u, v);
                CLIP(g);
                b = CONVERT_YUV_B(y[cnt], u);
                CLIP(b);
                rgbBuffer[(rowCnt * width + colCnt + cnt) * 3U + 0U] = (uint8_t)r;
                rgbBuffer[(rowCnt * width + colCnt + cnt) * 3U + 1U] = (uint8_t)g;
                rgbBuffer[(rowCnt * width + colCnt + cnt) * 3U + 2U] = (uint8_t)b;
            }
        }
        uvStart += width * (rowCnt % 2U);
    }
}

/*-----------------------------------------------------------------------------
 *      convert_YUV_420_422:  4:2:0 YUV to 4:2:2 YUV 
 *      
 *
 *  Parameters: uint8_t *Cin, uint8_t *Cout, uint16_t Clen
 *  Return:    int8_t
 *----------------------------------------------------------------------------*/
int8_t convert_YUV_420_422( uint8_t *Cin, uint8_t *Cout, uint16_t Clen)
{
   uint16_t i=0U;
   
   if ((Cin==NULL) || (Cout==NULL))
      return -1;
   while (i++<=Clen)
   {
      Cout[2U*i] = Cin[i];
      Cout[2U*i+1] = std::min(std::max(((9U * (Cin[i] + Cin[i+1U]) - (Cin[i-1U] + Cin[i+2U]) + 8U) >> 4U),0U),255U);
   }
   return 0;
}

/*-----------------------------------------------------------------------------
 *      NV12_YUV420P:  nv12 to YUV420P 
 *      image_src is the source image, image_dst is the converted image
 *
 *  Parameters: const unsigned char* image_src, unsigned char* image_dst, int16_t image_width, int16_t image_height
 *  Return:    void
 *----------------------------------------------------------------------------*/
void NV12_YUV420P(const unsigned char* image_src, unsigned char* image_dst, int16_t image_width, int16_t image_height)
{
        const unsigned char* pNV;
        unsigned char* pU;
        unsigned char* pV;
        int16_t i;
        
        unsigned char* p = image_dst;
        memcpy(p, image_src,(int16_t) (image_width * image_height * 3 / 2));
        p[(int16_t) (image_width * image_height * 3 / 2)] = '\0';               /* terminate the copy   */
        pNV = image_src + image_width * image_height;
        pU = p + image_width * image_height;
        pV = p + image_width * image_height + ((image_width * image_height)>>2U);
        for (i=0; i<(image_width * image_height)/2; i++)   
        {
           if ((i%2)==0) *pU++ = *(pNV + i);
           else *pV++ = *(pNV + i);
        }
}

/*-----------------------------------------------------------------------------
 *      RGB2HSL:   rgb to hsl 
 *
 *
 *  Parameters: uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *h, uint8_t *s, uint8_t *l
 *  Return:    void
 *----------------------------------------------------------------------------*/
void RGB2HSL(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *h, uint8_t *s, uint8_t *l )
{
   double themin,themax,delta;

   themin = std::min(*r, std::min(*g,*b));
   themax = std::max(*r, std::max(*g,*b));
   delta = themax - themin;
   *l = (themin + themax) / 2.0f;
   *s = 0.0f;
   if (*l > 0.0f && *l < 1.0f)
      *s = delta / (*l < 0.5f ? (2.0f* *l) : (2.0f-2.0f* *l));
   *h = 0.0f;
   if (delta > 0.0f) {
      if (themax == *r && themax != *g)
         *h += (*g - *b) / delta;
      if (themax == *g && themax != *b)
         *h += (2.0f + (*b - *r) / delta);
      if (themax == *b && themax != *r)
         *h += (4.0f + (*r - *g) / delta);
      *h *= 60.0f;
   }
}

/*  RGB 8bit 888 converrsion to compressed formats below
	RGB565	RGB332
R	5bit	3bit
G	6bit	3bit
B	5bit	2bit
Total number of bits	16bit	8bit
Representable colors	65536 colors	256 colors
example	0xFFFF	0xFF
*/

uint8_t convertRGBtoRGB332byte(uint8_t r, uint8_t g, uint8_t b)
{
    return (r & 0xE0) | ((g & 0xE0) >> 3) | ((b & 0xC0) >> 6);
}

// alternative way of writing it
uint8_t convertRGB888toRGB332byte( uint8_t r, uint8_t g, uint8_t b) {
    uint8_t newR = r>>5;
    uint8_t newG = g>>5;
    uint8_t newB = b>>6;
    return (newR<<5) | (newG<<2) | newB;
}

uint8_t convertRGB888toRGB565byte( uint8_t r, uint8_t g, uint8_t b) {
    uint8_t newR = r>>3;
    uint8_t newG = g>>2;
    uint8_t newB = b>>3;
    return (newR<<11) | (newG<<5) | newB;
}

void convertRGB888To332(uint8_t *src, uint8_t *dst, uint32_t pixelSize)
{
	for(uint32_t i = 0; i < pixelSize; i++){
		uint8_t r = (*src++);
		uint8_t g = (*src++);
		uint8_t b = (*src++);
		*dst++ = convertRGB888toRGB332byte(r, g, b);
	}
}

// doesnt work very well as we are trying to expand downsampled compressed data
void convertRGB332To888(uint8_t *src, uint8_t *dst, uint32_t pixelSize)
{
	float bias = 1.1f;                                                    // because downsampled this seemed to work better
	for(uint32_t i = 0; i < pixelSize; i++){
            uint8_t newRGB = (*src++);
            uint8_t newB = (newRGB & 0x3); 
            uint8_t newG = (newRGB & 0x1C) >> 2;
            uint8_t newR = (newRGB & 0xE0) >> 5;
            uint8_t r = newR << 5;
            uint8_t g = newG << 5;
            uint8_t b = newB << 6;
	    (*dst++) = static_cast<uint8_t>(round(r*bias));
	    (*dst++) = static_cast<uint8_t>(round(g*bias));
	    (*dst++) = static_cast<uint8_t>(round(b*bias));
	}
}


/*-----------------------------------------------------------------------------
 *      average_image:  average the image
 *
 *
 *  Parameters: int16_t w, int16_t h, double *img, double *img_avg
 *
 *  Return:     void
 *----------------------------------------------------------------------------*/
void average_image(int16_t w, int16_t h, double *img, double *img_avg)
{
        int16_t v,u;

        double pix;
        double *img_avg = img;
        for (v = 1; v < h-1; v++)
        {
            for (u = 1; u < w-1; u++)
            {
                pix = *img[u-1+(v-1)*w] + *img[u+(v-1)*w] + *img[u+1+(v-1)*w] + *img[u-1+v*w] + *img[u+v*w] + *img[u+1+v*w] + *img[u-1+(v+1)*w] + *img[u+(v+1)*w] + *img[u+1+(v+1)*w];
                *img_avg[u+v*w] = pix/9;
            }
        }
        return *img_avg;
}
