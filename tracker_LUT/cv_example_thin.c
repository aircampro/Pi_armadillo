/*
   compiles as  g++-8 -std=c++17 -o cv_thin cv_example_thin.c `pkg-config --cflags opencv` `pkg-config --libs opencv`
*/
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
 
void thinningIte(IplImage *Img_nichi, int pattern){
    IplImage *Img_sai = cvCreateImage(cvGetSize(Img_nichi), IPL_DEPTH_8U, 1);
    CvScalar ones = {1};
    cvSet(Img_sai, ones, NULL);
 
    for (int y = 1; y<Img_nichi->height - 1; y++){
        for (int x = 1; x<Img_nichi->width - 1; x++){
            int v9, v2, v3;
            int v8, v1, v4;
            int v7, v6, v5;
 
            v1 = CV_IMAGE_ELEM(Img_nichi, uchar, y + 0, x + 0);
            v2 = CV_IMAGE_ELEM(Img_nichi, uchar, y - 1, x + 0);
            v3 = CV_IMAGE_ELEM(Img_nichi, uchar, y - 1, x + 1);
            v4 = CV_IMAGE_ELEM(Img_nichi, uchar, y + 0, x + 1);
            v5 = CV_IMAGE_ELEM(Img_nichi, uchar, y + 1, x + 1);
            v6 = CV_IMAGE_ELEM(Img_nichi, uchar, y + 1, x + 0);
            v7 = CV_IMAGE_ELEM(Img_nichi, uchar, y + 1, x - 1);
            v8 = CV_IMAGE_ELEM(Img_nichi, uchar, y + 0, x - 1);
            v9 = CV_IMAGE_ELEM(Img_nichi, uchar, y - 1, x - 1);
 
            int S = (v2 == 0 && v3 == 1) + (v3 == 0 && v4 == 1) +
                    (v4 == 0 && v5 == 1) + (v5 == 0 && v6 == 1) +
                    (v6 == 0 && v7 == 1) + (v7 == 0 && v8 == 1) +
                    (v8 == 0 && v9 == 1) + (v9 == 0 && v2 == 1);
 
            int N = v2 + v3 + v4 + v5 + v6 + v7 + v8 + v9;
 
            int m1 = 0, m2 = 0;
 
            if (pattern == 0) m1 = (v2 * v4 * v6);
            if (pattern == 1) m1 = (v2 * v4 * v8);
 
            if (pattern == 0) m2 = (v4 * v6 * v8);
            if (pattern == 1) m2 = (v2 * v6 * v8);
 
            if (S == 1 && (N >= 2 && N <= 6) && m1 == 0 && m2 == 0)
                CV_IMAGE_ELEM(Img_sai, uchar, y, x) = 0;
        }
    }
    cvAnd(Img_nichi, Img_sai, Img_nichi, NULL);
    cvReleaseImage(&Img_sai);
}
 
IplImage *thinning(IplImage *Img_1){
    IplImage *white = cvCreateImage(cvGetSize(Img_1), IPL_DEPTH_8U,1);
    CvScalar full_bits = {0xff};
    cvSet(white, full_bits, NULL);
 
    IplImage *Img_2 = cvCloneImage(Img_1);
    cvDiv(Img_2, white, Img_2, 1);
 
    IplImage *prev = cvCreateImage(cvGetSize(Img_2), IPL_DEPTH_8U, 1);
    IplImage *diff = cvCreateImage(cvGetSize(Img_2), IPL_DEPTH_8U, 1);
 
    do {
        thinningIte(Img_2, 0);
        thinningIte(Img_2, 1);
        cvAbsDiff(Img_2, prev, diff);
        cvCopy(Img_2, prev, NULL);
    } while (cvCountNonZero(diff) > 0);
 
    cvMul(Img_2, white, Img_2, 1);
 
    cvReleaseImage(&white);
    cvReleaseImage(&diff);
    cvReleaseImage(&prev);
    return Img_2;
}
 
int main(int argc, char* argv[]){
    if (argc < 2) return 1;
 
    IplImage *Img_orig; 
    IplImage *Img_gray; 
    IplImage *Img_nichi; // threshold
    IplImage *Img_sai;   // thinned
 
    int W, H; //widthとheight
 
    //load command line picture
    Img_orig = cvLoadImage(argv[1], 1);
 
    W = Img_orig->width;//幅
    H = Img_orig->height;//高さ
 
    //convert to gray create the image headers
    Img_gray  = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1); 
    Img_nichi = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1); 
 
    //to gray scale
    cvCvtColor(Img_orig, Img_gray, CV_BGR2GRAY);
 
    //threshold
    cvThreshold(Img_gray, Img_nichi, 10, 255, CV_THRESH_BINARY);
 
    //apply thinning
    Img_sai = thinning(Img_nichi);
 
    //show the images
    //cvShowImage("元画像", Img_orig);
    //cvShowImage("グレースケール", Img_gray);
    //cvShowImage("2値 (細線化前)", Img_nichi);
    //cvShowImage("2値 (細線化後)", Img_sai);
 
    //save the images
    cvSaveImage("No_5a.bmp", Img_gray, 0);
    cvSaveImage("No_5b.bmp", Img_nichi, 0);
    cvSaveImage("No_5c.bmp", Img_sai, 0);
 
    //wait
    cvWaitKey(0);
 
    //kill the windows
    cvDestroyAllWindows();
 
    //release the image(s)
    cvReleaseImage(&Img_orig);
    cvReleaseImage(&Img_gray);
    cvReleaseImage(&Img_nichi);
    cvReleaseImage(&Img_sai);
 
    return 0;
}