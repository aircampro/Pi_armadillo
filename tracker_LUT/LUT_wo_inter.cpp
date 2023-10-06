//
// LUT colour correcton code without interpolation
//
// ref:- https://shizenkarasuzon.hatenablog.com/entry/2020/08/13/185223
//
# include <opencv2/opencv.hpp>
# include <opencv2/highgui.hpp>
# include <iostream>
#include <vector>
#include <string>
#include <fstream>

#define DISP(X) (std::cout << #X << " = "  << X << std::endl)

// https://qiita.com/iseki-masaya/items/70b4ee6e0877d12dafa8 を参考にしました
std::vector<std::string> split_naive(const std::string &s) {
  std::vector<std::string> elems;
  std::string item;
  for (char ch : s) {
    if (ch == ' ') {
      if (!item.empty())
        elems.push_back(item);
      item.clear();
    }else {item += ch; }
  }
  if (!item.empty())
    elems.push_back(item);
  return elems;
}

#define LUT_SIZE 33

class my3D_lut_simple{
private:
  int lut_size;
  cv::Mat src;
  cv::Mat dst;
  unsigned char lut_data[LUT_SIZE * LUT_SIZE * LUT_SIZE][3];

public:
  // read .cube file
  int read_lut_file(std::string filename) {
    std::ifstream ifs(filename);
    std::string one_line;
    
    int i = 0;
    while(i < LUT_SIZE * LUT_SIZE * LUT_SIZE){
      getline(ifs, one_line);
      std::vector<std::string> values = split_naive(one_line);
      

      if (values.size() != 3) { std::cout << one_line << " --> " << values.size() << std::endl; continue; }

      lut_data[i][0] = ::atof(values[0].c_str()) * 255;
      lut_data[i][1] = ::atof(values[1].c_str()) * 255;
      lut_data[i][2] = ::atof(values[2].c_str()) * 255;
      i++;
    }
	return 1;
  }

  void setSrcImg(cv::Mat src_img) { 
    src = src_img;    
    dst = cv::Mat::zeros(src.size(), src.type());
  }

  void convert() {
    if (src.empty()) { std::cout << "src image is empty!"; return; }

    int x, y, index;
    int r, g, b;
    for (y = 0; y < src.rows; y++) {
      cv::Vec3b* p_src = src.ptr<cv::Vec3b>(y);
      cv::Vec3b* p_dst = dst.ptr<cv::Vec3b>(y);

      for (x = 0; x < src.cols; x++) {
        r = float(p_src[x][0]) * LUT_SIZE / 256.0f;
        g = float(p_src[x][1]) * LUT_SIZE / 256.0f;
        b = float(p_src[x][2]) * LUT_SIZE / 256.0f; 
        index = r + g * LUT_SIZE + b*LUT_SIZE * LUT_SIZE;

        assert(index > 35936);

        p_dst[x][0] = lut_data[index][0];
        p_dst[x][1] = lut_data[index][1];
        p_dst[x][2] = lut_data[index][2];
      }
    }
  }

  cv::Mat getResultImg() {
    return dst;
  }
};


int main() {
  my3D_lut_simple myLut;
  cv::Mat img = cv::imread("/home/anthony/usb_test/test.JPG");
  myLut.setSrcImg(img);
  myLut.read_lut_file("test.cube");

  std::cout << "convert start!\n";
  myLut.convert();
  std::cout << "convert finished!\n";

  cv::Mat dst = myLut.getResultImg();

  // show the LUT corrected image
  //cv::imshow("test", dst);
  
  // write the LUT corrected image
  cv::imwrite("test_LUT.jpg", dst);
  
  cv::waitKey(0);

  return 0;
}