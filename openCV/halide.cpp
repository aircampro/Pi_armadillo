# include <opencv2/opencv.hpp>
# pragma comment(lib, "opencv_core331.lib")
# pragma comment(lib, "opencv_imgcodecs331.lib")
# pragma comment(lib, "opencv_highgui331.lib")
# include "include/Halide.h"
# pragma comment(lib, "Halide.lib")
using namespace Halide;

//OpenCV for Halide functions
Buffer<uint8_t> imread(cv::String name);
void imwrite(cv::String name, const Buffer<uint8_t>& src);
void convertHalide2Mat(const Buffer<uint8_t>& src, cv::Mat& dest);
void convertMat2Halide(cv::Mat& src, Buffer<uint8_t>& dest);
void imshow(cv::String name, const Buffer<uint8_t>& src);
void imshow16(cv::String name, const Buffer<int16_t>& src, double offset = 0.0, double scale = 1.0);
void guiAlphaBlend(Buffer<uint8_t>& src1, Buffer<uint8_t>& src2, cv::String name = "alpha blend");
void bilateral_filter_opencv(cv::Mat& src, cv::Mat& dest, int rad, float sigma_s, float sigma_c);

Func Sobel_filter_h(Buffer<uint8_t>& src)
{
	Var x("x"), y("y"), c("c");
	Func clamped = BoundaryConditions::repeat_edge(src);

	Func input_16("input_int16");
	input_16(x, y, c) = cast<int16_t>(clamped(x, y, c));

	Func output("output");
	output(x, y, c) = -input_16(x - 1, y - 1, c) + input_16(x + 1, y - 1, c)
		- 2 * input_16(x - 1, y, c) + 2 * input_16(x + 1, y, c)
		- input_16(x - 1, y + 1, c) + input_16(x + 1, y + 1, c);

	return output;
}

Func box_filter1D(Buffer<uint8_t>& src, int rad)
{
	Var x("x"), y("y"), c("c");
	Func clamped = BoundaryConditions::repeat_edge(src);

	Func input_32("input_32");
	input_32(x, y, c) = cast<uint32_t>(clamped(x, y, c));

	RDom r(-rad, 2 * rad + 1);

	Func blur("blur");
	blur(x, y, c) += input_32(x + r, y, c);

	Func output("output");
	output(x, y, c) = cast<uint8_t>(blur(x, y, c) / (2 * rad + 1));

	return output;
}

Func box_filter1D_sum(Buffer<uint8_t>& src, int rad)
{
	Var x("x"), y("y"), c("c");
	Func clamped = BoundaryConditions::repeat_edge(src);

	Func input_32("input_32");
	input_32(x, y, c) = cast<uint32_t>(clamped(x, y, c));

	RDom r(-rad, 2 * rad + 1);

	Func output("output");
	output(x, y, c) = cast<uint8_t>(sum(input_32(x + r, y, c)) / (2 * rad + 1));

	return output;
}

Func box_filter(Buffer<uint8_t>& src, int rad)
{
	Var x("x"), y("y"), c("c");
	Func clamped = BoundaryConditions::repeat_edge(src);

	Func input_32("input_32");
	input_32(x, y, c) = cast<uint32_t>(clamped(x, y, c));

	RDom r(-rad, 2 * rad + 1, -rad, 2 * rad + 1);

	Func blur("blur");
	blur(x, y, c) += input_32(x + r.x, y + r.y, c);

	Func output("output");
	output(x, y, c) = cast<uint8_t>(blur(x, y, c) / ((2 * rad + 1)*(2 * rad + 1)));

	return output;
}

Func box_filter_sum(Buffer<uint8_t>& src, int rad)
{
	Var x("x"), y("y"), c("c");
	Func clamped = BoundaryConditions::repeat_edge(src);

	Func input_32("input_32");
	input_32(x, y, c) = cast<uint32_t>(clamped(x, y, c));

	RDom r(-rad, 2 * rad + 1, -rad, 2 * rad + 1);

	Func output("output");
	output(x, y, c) = cast<uint8_t>(sum(input_32(x + r.x, y + r.y, c)) / ((2 * rad + 1)*(2 * rad + 1)));

	return output;
}

Func Gaussian_filter(Buffer<uint8_t>& src, int rad, float sigma)
{
	Var x("x"), y("y"), c("c");
	Func clamped = BoundaryConditions::repeat_edge(src);

	Func input_float("input_float");
	input_float(x, y, c) = cast<float>(clamped(x, y, c));

	RDom r(-rad, 2 * rad + 1, -rad, 2 * rad + 1);

	Func blur("blur");
	Expr d = -1.f / (2.f*sigma*sigma);
	Expr total = sum(fast_exp((r.x*r.x + r.y*r.y) *d));

	blur(x, y, c) += fast_exp((r.x*r.x + r.y*r.y)*d)*input_float(x + r.x, y + r.y, c);

	Func output("output");
	output(x, y, c) = cast<uint8_t>(blur(x, y, c) / total);

	return output;
}

Func bilateral_filter_gray(Buffer<uint8_t>& src, int rad, float sigma_s, float sigma_c)
{
	Var x("x"), y("y"), c("c");
	Func clamped = BoundaryConditions::repeat_edge(src);

	Func input_float("input_float");
	input_float(x, y, c) = cast<float>(clamped(x, y, c));

	RDom r(-rad, 2 * rad + 1, -rad, 2 * rad + 1);

	Func blur("blur");
	Expr ds = -1.f / (2.f*sigma_s*sigma_s);
	Expr dc = -1.f / (2.f*sigma_c*sigma_c);
	Expr total = sum(
		fast_exp((r.x*r.x + r.y*r.y)*ds)
		*fast_exp(
		(input_float(x + r.x, y + r.y, c) - input_float(x, y, c))*(input_float(x + r.x, y + r.y, c) - input_float(x, y, c))
			//fast_pow((input_float(x + r.x, y + r.y, c) - input_float(x, y, c)),2)
			* dc)
	);

	blur(x, y, c) += fast_exp((r.x*r.x + r.y*r.y)*ds)
		*fast_exp(
		(input_float(x + r.x, y + r.y, c) - input_float(x, y, c))*(input_float(x + r.x, y + r.y, c) - input_float(x, y, c))
			//fast_pow((input_float(x + r.x, y + r.y, c) - input_float(x, y, c)),2)
			*dc)
		*input_float(x + r.x, y + r.y, c);

	Func output("output");
	output(x, y, c) = cast<uint8_t>(blur(x, y, c) / total);

	return output;
}

Func bilateral_filter_color(Buffer<uint8_t>& src, int rad, float sigma_s, float sigma_c)
{
	Var x("x"), y("y"), c("c");
	Func clamped = BoundaryConditions::repeat_edge(src);

	Func input_float("input_float");
	input_float(x, y, c) = cast<float>(clamped(x, y, c));

	Func blur("blur");
	Expr ds = -1.f / (2.f*sigma_s*sigma_s);
	Expr dc = -1.f / (2.f*sigma_c*sigma_c);

	RDom r(-rad, 2 * rad + 1, -rad, 2 * rad + 1);

	Expr total = sum(
		fast_exp((r.x*r.x + r.y*r.y)*ds)
		*fast_exp(
		(
			(input_float(x + r.x, y + r.y, 0) - input_float(x, y, 0))*(input_float(x + r.x, y + r.y, 0) - input_float(x, y, 0))
			+ (input_float(x + r.x, y + r.y, 1) - input_float(x, y, 1))*(input_float(x + r.x, y + r.y, 1) - input_float(x, y, 1))
			+ (input_float(x + r.x, y + r.y, 2) - input_float(x, y, 2))*(input_float(x + r.x, y + r.y, 2) - input_float(x, y, 2))
			)
			* dc)
	);

	blur(x, y, c) +=
		fast_exp((r.x*r.x + r.y*r.y)*ds)
		*fast_exp(
		(
			(input_float(x + r.x, y + r.y, 0) - input_float(x, y, 0))*(input_float(x + r.x, y + r.y, 0) - input_float(x, y, 0))
			+ (input_float(x + r.x, y + r.y, 1) - input_float(x, y, 1))*(input_float(x + r.x, y + r.y, 1) - input_float(x, y, 1))
			+ (input_float(x + r.x, y + r.y, 2) - input_float(x, y, 2))*(input_float(x + r.x, y + r.y, 2) - input_float(x, y, 2))
			)
			*dc)
		*input_float(x + r.x, y + r.y, c);



	Func output("output");
	output(x, y, c) = cast<uint8_t>(blur(x, y, c) / total);

	return output;
}


//utility function with OpenCV
void convertMat2Halide(cv::Mat& src, Buffer<uint8_t>& dest)
{
	const int ch = src.channels();
	if (ch == 1)
	{
		for (int j = 0; j < src.rows; j++)
		{
			for (int i = 0; i < src.cols; i++)
			{
				dest(i, j) = src.at<uchar>(j, i);
			}
		}
	}
	else if (ch == 3)
	{
		for (int j = 0; j < src.rows; j++)
		{
			for (int i = 0; i < src.cols; i++)
			{
				dest(i, j, 0) = src.at<uchar>(j, 3 * i);
				dest(i, j, 1) = src.at<uchar>(j, 3 * i + 1);
				dest(i, j, 2) = src.at<uchar>(j, 3 * i + 2);
			}
		}
	}
}

Buffer<uint8_t> imread(cv::String name)
{
	cv::Mat a = cv::imread(name);
	if (a.empty()) std::cout << name << " is empty" << std::endl;

	Buffer<uint8_t> ret(a.cols, a.rows, a.channels());
	convertMat2Halide(a, ret);

	return ret;
}

void convertHalide2Mat(const Buffer<uint8_t>& src, cv::Mat& dest)
{
	if (dest.empty()) dest.create(cv::Size(src.width(), src.height()), CV_MAKETYPE(CV_8U, src.channels()));
	const int ch = dest.channels();
	if (ch == 1)
	{
		for (int j = 0; j < dest.rows; j++)
		{
			for (int i = 0; i < dest.cols; i++)
			{
				dest.at<uchar>(j, i) = src(i, j);
			}
		}
	}
	else if (ch == 3)
	{
		for (int j = 0; j < dest.rows; j++)
		{
			for (int i = 0; i < dest.cols; i++)
			{
				dest.at<uchar>(j, 3 * i + 0) = src(i, j, 0);
				dest.at<uchar>(j, 3 * i + 1) = src(i, j, 1);
				dest.at<uchar>(j, 3 * i + 2) = src(i, j, 2);
			}
		}
	}
}

void imwrite(cv::String name, const Buffer<uint8_t>& src)
{
	cv::Mat a(cv::Size(src.width(), src.height()), CV_MAKETYPE(CV_8U, src.channels()));
	convertHalide2Mat(src, a);
	cv::imwrite(name, a);
}

void imshow(cv::String name, const Buffer<uint8_t>& src)
{
	cv::Mat a(cv::Size(src.width(), src.height()), CV_MAKETYPE(CV_8U, src.channels()));
	convertHalide2Mat(src, a);
	cv::imshow(name, a);
}

void imshow16(cv::String name, const Buffer<int16_t>& src, double offset, double scale)
{
	cv::Mat a(cv::Size(src.width(), src.height()), CV_MAKETYPE(CV_8U, src.channels()));

	const int ch = a.channels();
	if (ch == 1)
	{
		for (int j = 0; j < a.rows; j++)
		{
			for (int i = 0; i < a.cols; i++)
			{
				a.at<uchar>(j, i) = cv::saturate_cast<uchar>(scale*src(i, j) + offset);
			}
		}
	}
	else if (ch == 3)
	{
		for (int j = 0; j < a.rows; j++)
		{
			for (int i = 0; i < a.cols; i++)
			{

				a.at<uchar>(j, 3 * i + 0) = cv::saturate_cast<uchar>(scale*src(i, j, 0) + +offset);
				a.at<uchar>(j, 3 * i + 1) = cv::saturate_cast<uchar>(scale*src(i, j, 1) + +offset);
				a.at<uchar>(j, 3 * i + 2) = cv::saturate_cast<uchar>(scale*src(i, j, 2) + +offset);
			}
		}
	}

	cv::imshow(name, a);
}

void guiAlphaBlend(Buffer<uint8_t>& src1, Buffer<uint8_t>& src2, cv::String name)
{
	cv::Mat s1(cv::Size(src1.width(), src1.height()), CV_MAKETYPE(CV_8U, src1.channels()));
	cv::Mat s2(cv::Size(src1.width(), src1.height()), CV_MAKETYPE(CV_8U, src1.channels()));
	convertHalide2Mat(src1, s1);
	convertHalide2Mat(src2, s2);

	cv::namedWindow(name);
	int a = 0; cv::createTrackbar("alpha", name, &a, 100);
	int key = 0;
	while (key != 'q')
	{
		cv::Mat show;
		cv::addWeighted(s1, 1.0 - a / 100.0, s2, a / 100.0, 0.0, show);
		cv::imshow(name, show);
		key = cv::waitKey(1);
	}
	cv::destroyWindow(name);
}

void bilateral_filter_opencv(cv::Mat& src, cv::Mat& dest, int rad, float sigma_s, float sigma_c)
{
	if (dest.empty())dest.create(src.size(), src.type());
	cv::Mat im;
	cv::copyMakeBorder(src, im, rad, rad, rad, rad, cv::BORDER_REFLECT);

	float ss = -1.f / (2.f*sigma_s*sigma_s);
	float sc = -1.f / (2.f*sigma_c*sigma_c);
# pragma omp parallel for
	for (int j = 0; j < src.rows; j++)
	{
		for (int i = 0; i < src.cols; i++)
		{
			float tw = 0.f;
			float tb = 0.f;
			float tg = 0.f;
			float tr = 0.f;

			float rb = im.at<uchar>(j + rad, 3 * (i + rad) + 0);
			float rg = im.at<uchar>(j + rad, 3 * (i + rad) + 1);
			float rr = im.at<uchar>(j + rad, 3 * (i + rad) + 2);
			for (int l = -rad; l <= rad; l++)
			{
				uchar* s = im.ptr<uchar>(j + l + rad) + 3 * (i + rad);
				for (int k = -rad; k <= rad; k++)
				{
					float diff =
						(rb - s[3 * k + 0])*(rb - s[3 * k + 0])
						+ (rg - s[3 * k + 1])*(rg - s[3 * k + 1])
						+ (rr - s[3 * k + 2])*(rr - s[3 * k + 2]);
					float w = exp((k*k + l*l)*ss)*exp(diff*sc);
					tb += w*s[3 * k + 0];
					tg += w*s[3 * k + 1];
					tr += w*s[3 * k + 2];
					tw += w;
				}
			}
			dest.at<uchar>(j, 3 * i + 0) = cv::saturate_cast<uchar>(tb / tw);
			dest.at<uchar>(j, 3 * i + 1) = cv::saturate_cast<uchar>(tg / tw);
			dest.at<uchar>(j, 3 * i + 2) = cv::saturate_cast<uchar>(tr / tw);
		}
	}
}

int main(int argc, char **argv)
{
	Buffer<uint8_t> input = imread("rgb.png");

	Func output1 = Sobel_filter_h(input);
	Buffer<int16_t> result1 = output1.realize(input.width(), input.height(), 3);
	imshow16("Sobel", result1, 0);

	Func output2_ = box_filter1D(input, 17);
	Buffer<uint8_t> result2_ = output2_.realize(input.width(), input.height(), 3);
	imshow("box1D", result2_);

	Func output2__ = box_filter1D_sum(input, 17);
	Buffer<uint8_t> result2__ = output2__.realize(input.width(), input.height(), 3);
	imshow("box1DSum", result2__);

	Func output2 = box_filter(input, 17);
	Buffer<uint8_t> result2 = output2.realize(input.width(), input.height(), 3);
	imshow("box", result2);

	Func output3 = Gaussian_filter(input, 17, 4.f);
	Buffer<uint8_t> result3 = output3.realize(input.width(), input.height(), 3);
	imshow("Gaussian", result3);

	Func output4 = bilateral_filter_gray(input, 9, 4.f, 50.f);
	Buffer<uint8_t> result4 = output4.realize(input.width(), input.height(), 3);
	imshow("Bilateral gray", result4);

	Func output5 = bilateral_filter_color(input, 7, 4.f, 30.f);
	Buffer<uint8_t> result5 = output5.realize(input.width(), input.height(), 3);
	imshow("Bilateral color", result5);

	//test for bilateral filter
	cv::Mat imat, dest;
	convertHalide2Mat(input, imat);
	bilateral_filter_opencv(imat, dest, 7, 4.f, 30.f);
	Buffer<uint8_t> ocv(dest.cols, dest.rows, dest.channels());
	convertMat2Halide(dest, ocv);
	guiAlphaBlend(ocv, result5, "bilateral filter compare");
	//cv::waitKey();

	return 0;
}
