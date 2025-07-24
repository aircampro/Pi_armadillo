// simple openCV screen using c++20 format to draw out your inofrmation then display it
//
#include "opencv2/opencv.hpp"
#include <format>
#include <iostream>

# example class to store the information (you would have populated from another interface)
class person {
	// person class with variables declaration as private
private:
	int id;
	int age;
	std::string name;
public:
	void
	print(){
		std::cout << "id:"   << this->id << std::endl;
		std::cout << "age:"  << this->age << std::endl;
		std::cout << "name:" << this=>name << std::endl;
	}
	int
	get_age(){
	    return this->age;
    }
	int
	get_id(){
        return this->id;
    }
    string
    get_name(){
        return this->name;
    }
}

int main()
{
    const int start_row = 70;
    const int text_ht = 30;
    const int text_in = 30;
    cv::Mat outImage = cv::Mat::zeros(300, 300, CV_8UC3);
	
    // string to screen
    person peter(0, 21, "peter");
    std::string message = std::format("My name is {} and I am {} years old.", peter.get_name(), peter.get_age());
    cv::putText(outImage, message, cv::Point(text_in, start_row), cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(0, 200, 0));

	
    // binary 
    int numericval = 0x34;
    std::string message = std::format("Hex: {0:x}, Oct: {0:o}, Bin: {0:b}", numericval);
    cv::putText(outImage, message, cv::Point(text_in, start_row+text_ht), cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(200, 0, 0));
	
    // cyclic char buffer written to and put to the to screen
    std::vector<char> buffer;
    buffer.reserve(100);
    person tom(1, 25, "tommy");
    person john(2, 17, "jack");
    auto it = std::format_to(std::back_inserter(buffer), "Hello, {}!", tom.get_name());
    buffer.push_back('\0');
    auto it = std::format_to(std::back_inserter(buffer), "Hello, {}!", john.get_name());
    buffer.push_back('\0');
    cv::putText(outImage, buffer.data(), cv::Point(text_in, start_row+(text_ht*2)), cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(0, 0, 200));

    cv::imshow("example", outImage);
    cv::waitKey(0);
    return 0;
}