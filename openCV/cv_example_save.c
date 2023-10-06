#include <cv.h>
#include <highgui.h>

#define Rgb(Image, u,v) \
 (*( (unsigned char*)Image->imageData +2+ 3*(u)+(v)*Image->widthStep ) )
#define rGb(Image, u,v) \
 (*( (unsigned char*)Image->imageData +1+ 3*(u)+(v)*Image->widthStep ) )
#define rgB(Image, u,v) \
 (*( (unsigned char*)Image->imageData +0+ 3*(u)+(v)*Image->widthStep ) )
 
main()
{
 int u,v, R,G,B;
 IplImage* Image;

 Image = cvLoadImage("Image.PPM", CV_LOAD_IMAGE_COLOR);

 for( v=0; v< Image->height; v++ ) for( u=0; u< Image->width; u++ ) {
 R = Rgb(Image, u, v); G = rGb(Image, u, v); B = rgB(Image, u, v);
 if( R>200 && G<170 && B<170 ){
 Rgb(Image, u,v) = (R+G+B)/3; rGb(Image, u,v) = rgB(Image, u,v) = 0;
 }
 else {
 Rgb(Image, u,v) = rGb(Image, u,v) = rgB(Image, u,v) = (R+G+B)/3;
 }
 }

 cvSaveImage( "red.ppm", Image, 0 );

 cvReleaseImage( &Image );
}