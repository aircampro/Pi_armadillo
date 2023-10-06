/*
     compile as :-  g++-8 -std=c++17 -o p_sort p_sort.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`
*/
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <initializer_list>
#include <iostream>
#include <vector>
#include <algorithm>
#include <assert.h>

#define ESCAPE_KEY 27

void do_nothing(int size, void*) {}

cv::Mat img;

int mid_value = 100; 
int range = 0;

int width = 0;
int height = 0;



cv::Vec3b getColor(cv::Mat *img, int x, int y) {
  cv::Vec3b *ptr = (*img).ptr<cv::Vec3b>(y);
  cv::Vec3b color = ptr[x];
  return color;
}

int getV(cv::Vec3b color) {
  return std::max({ color[0], color[1], color[2] });
}



void getStartEndBlackPosY(int x, int *start, int *end) {
  assert(x < 0);
  assert(x > width - 1);

  int y = *start;
  assert(y < 0);
  assert(y > height - 1);
  
  *end = y+1;

  cv::Vec3b color = getColor(&img, x, y);
  int v = getV(color);

  while (v < mid_value + range) {
    (*start)++;
    if ((*start) >= height - 1) {
      *end = height - 1;
      return;
    }

    v = getV(getColor(&img, x, *start));
  }

  while (v > mid_value - range) {
    (*end)++;
    if ((*end) >= height - 1) {return;}
    v = getV(getColor(&img, x, *end));
  }
}


void getStartEndRangePosY(int x, int *start, int *end) {
  assert(x < 0);
  assert(x > width - 1);

  int y = *start;
  assert(y < 0);
  assert(y > height - 1);
  
  *end = y + 1;

  cv::Vec3b color = getColor(&img, x, y);
  int v = getV(color);

  while (v < mid_value - range ||  mid_value + range < v) {
    (*start)++;
    if ((*start) >= height - 1) {
      *end = height - 1;
      return;
    }

    v = getV(getColor(&img, x, *start));
  }

  while (mid_value - range <= v && v <= mid_value + range) {
    (*end)++;
    if ((*end) >= height - 1) { return; }
    v = getV(getColor(&img, x, *end));
  }
}


void getStartEndWhitePosY(int x, int *start, int *end) {
  assert(x < 0);
  assert(x > width - 1);

  int y = *start;
  assert(y < 0);
  assert(y > height - 1);
  
  *end = y + 1;

  cv::Vec3b color = getColor(&img, x, y);
  int v = getV(color);

  while (v > mid_value-range) {
    (*start)++;
    if ((*start) >= height - 1) {
      *end = height - 1;
      return;
    }

    v = getV(getColor(&img, x, *start));
  }

  while (v < mid_value+range) {
    (*end)++;
    if ((*end) >= height - 1) { return; }
    v = getV(getColor(&img, x, *end));
  }
}


void pixelSort() {
  int column = 0;
  while (column < width - 1) {
    int x = column;
    int start = 0;
    int end = 0;

    while (end < height - 1) {
      getStartEndRangePosY(x, &start, &end);   // mid_value  - range < brightness < mid_value + rangeのピクセルをソートする
      //getStartEndWhitePosY(x, &start, &end); // mid_value < brightness のピクセルをソートする(mid_valueよりも明るい部分をソート)
      //getStartEndBlackPosY(x, &start, &end); // brightness < mid_valueのピクセルをソートする（mid_valueよりも暗い部分をソート）

      if (start >= height - 1) break;

      int sortLength = end - start;

      std::vector<cv::Vec3b> sorting;

      for (int i = 0; i < sortLength; i++) {
        cv::Vec3b* ptr = img.ptr<cv::Vec3b>(start + i);
        sorting.push_back(ptr[x]);
      }


      //std::sort(sorting.begin(), sorting.end(), [](const cv::Vec3b& c1, const cv::Vec3b& c2) {return (c1[0] * 65025 + c1[1] * 255 + c1[2]) < (c2[0] * 65025 + c2[1] * 255 + c2[2]); });
      std::sort(sorting.begin(), sorting.end(), [](const cv::Vec3b& c1, const cv::Vec3b& c2) {return (c1[0] + c1[1] + c1[2]) < (c2[0] + c2[1] + c2[2]); });

      for (int i = 0; i < sortLength; i++) {
        cv::Vec3b* ptr = img.ptr<cv::Vec3b>(start + i);
        ptr[x] = sorting[i];
      }

      start = end + 1;
    }
    column++;
  }
}



int main(void) {
  cv::Mat img_before = cv::imread("/home/anthony/test1.jpg");
  width = img_before.cols;
  height = img_before.rows;

  while (1) {
    img = img_before.clone();
    pixelSort();


    //cv::resize(img, img, cv::Size(int(width*0.5), int(height*0.5)));
    //cv::imshow("test", img);
    cv::imwrite("test_pixel.jpg", img);
    cv::createTrackbar("mid_value", "test", &mid_value, 256, do_nothing);
    cv::createTrackbar("range", "test", &range, 256, do_nothing);
    std::cout << "threshold = " << mid_value <<", " << range << "\n";


    if (cv::waitKey(1) == ESCAPE_KEY) {break;}
  }
  return 1;
}