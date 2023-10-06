/*
      LUT with tri-linear interpolation
	  ref:- https://shizenkarasuzon.hatenablog.com/entry/2020/08/13/185223
	  
	  Compiles as :-  g++-8 -std=c++17 -o LUT_w_interp LUT_w_interp.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`
*/
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
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
      if (!item.empty()) elems.push_back(item);
      item.clear();
    }else { item += ch;}
  }
  if (!item.empty()) elems.push_back(item);
  return elems;
}

#define LUT_SIZE 33

/*
   class to perform LUT correction
*/
class my3D_lut {
private:
  int lut_size;
  cv::Mat src;
  cv::Mat dst;
  cv::Vec3f lut_data[LUT_SIZE * LUT_SIZE * LUT_SIZE];

public:
  // read .cube file
  int read_lut_file(std::string filename) {
    std::ifstream ifs(filename);
    std::string one_line;

    int i = 0;
    while (i < LUT_SIZE * LUT_SIZE * LUT_SIZE) {
      getline(ifs, one_line);
      std::vector<std::string> values = split_naive(one_line);

      if (values.size() != 3) { std::cout << one_line << " --> " << values.size() << std::endl; continue; }

      lut_data[i][0] = ::atof(values[0].c_str());
      lut_data[i][1] = ::atof(values[1].c_str());
      lut_data[i][2] = ::atof(values[2].c_str());
      i++;
    }
	return 1;
  }

  void setSrcImg(cv::Mat src_img) {
    src = src_img;
    dst = cv::Mat::zeros(src.size(), src.type());
  }

  cv::Vec3f _convert_pixel(cv::Vec3b color) {
    unsigned char pos[3]; // 0~33
    float delta[3]; //

    pos[0] = color[0] * LUT_SIZE / 256;
    pos[1] = color[1] * LUT_SIZE / 256;
    pos[2] = color[2] * LUT_SIZE / 256;

    delta[0] = float(color[0] * LUT_SIZE) / 256.0f - pos[0];
    delta[1] = float(color[1] * LUT_SIZE) / 256.0f - pos[1];
    delta[2] = float(color[2] * LUT_SIZE) / 256.0f - pos[2];


    cv::Vec3f vertex_color[8];
    cv::Vec3f surf_color[4];
    cv::Vec3f line_color[2];
    cv::Vec3f out_color;

    int index = pos[0] + pos[1] * LUT_SIZE + pos[2] * LUT_SIZE * LUT_SIZE;
    const int max_range = 33 * 33 * 33 - 1;

    unsigned int next_index[3] = { 1, LUT_SIZE, LUT_SIZE * LUT_SIZE };
    if (index % LUT_SIZE == LUT_SIZE - 1) { next_index[0] = 0; }
    if ((index/LUT_SIZE) % LUT_SIZE == LUT_SIZE - 1) { next_index[1] = 0; }
    if ((index/(LUT_SIZE * LUT_SIZE))% LUT_SIZE == LUT_SIZE - 1) {next_index[2] = 0;}

    // https://en.wikipedia.org/wiki/Trilinear_interpolation
    vertex_color[0] = lut_data[index];
    vertex_color[1] = lut_data[index + next_index[0]];
    vertex_color[2] = lut_data[index + next_index[0] + next_index[1]];
    vertex_color[3] = lut_data[index + next_index[1]];
    vertex_color[4] = lut_data[index + next_index[2]];
    vertex_color[5] = lut_data[index + next_index[0] + next_index[2]];
    vertex_color[6] = lut_data[index + next_index[0] + next_index[1] + next_index[2]];
    vertex_color[7] = lut_data[index + next_index[1] + next_index[2]];

    surf_color[0] = vertex_color[0] * (1.0f - delta[2]) + vertex_color[4] * delta[2];
    surf_color[1] = vertex_color[1] * (1.0f - delta[2]) + vertex_color[5] * delta[2];
    surf_color[2] = vertex_color[2] * (1.0f - delta[2]) + vertex_color[6] * delta[2];
    surf_color[3] = vertex_color[3] * (1.0f - delta[2]) + vertex_color[7] * delta[2];

    line_color[0] = surf_color[0] * (1.0f - delta[0]) + surf_color[1] * delta[0];
    line_color[1] = surf_color[2] * (1.0f - delta[0]) + surf_color[3] * delta[0];

    out_color = line_color[0] * (1.0f - delta[1]) + line_color[1] * delta[1];

    return out_color;
  }


  void convert() {
    if (src.empty()) { std::cout << "src image is empty!"; return; }
    int x, y, index;
    for (y = 0; y < src.rows; y++) {
      cv::Vec3b* p_src = src.ptr<cv::Vec3b>(y);
      cv::Vec3b* p_dst = dst.ptr<cv::Vec3b>(y);

      for (x = 0; x < src.cols; x++) {
        cv::Vec3f color_out = _convert_pixel(p_src[x]);
        p_dst[x] = cv::Vec3b(color_out[0] * 255.0, color_out[1] * 255.0, color_out[2] * 255.0 );
      }
    }
  }

  cv::Mat getResultImg() {return dst;}
};


int main() {
  my3D_lut myLut;
  cv::Mat img = cv::imread("/home/anthony/usb_test/test_scene.jpg");
  myLut.setSrcImg(img);
  myLut.read_lut_file("/home/anthony/usb_test/NIGHT_KING_141.Clip__45.cube");

  std::cout << "convert start!\n";
  myLut.convert();
  std::cout << "convert finished!\n";

  cv::Mat dst = myLut.getResultImg();

  // image show
  //cv::imshow("test", dst);
  
  // write the LUT corrected image
  cv::imwrite("test_LUT_TLI.jpg", dst);
  
  cv::waitKey(0);

  return 0;
}