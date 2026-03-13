/*

      Move Mitsubishi robot using joystick and if you click usb cam viewer write the rgb and hsv for the point to the control window
	  requires this library :- https://chitose-robotics.com/product
	  
*/
#include <iostream>
#include <optional>
#include <string>
#include "crewbo/crewbo.h"
#include "project_lib/joystick.h"
#include "project_lib/vision_assistant.h"
#include <math>
#include <opencv2/opencv.hpp>

// apply a deadzone on the axis
constexpr float DEADZONE = 0.05f;
static float applyDeadzone(float v)
{
    if (std::fabs(v) < DEADZONE)
        return 0.0f;
    return v;
}

// various algorythoms to use with speed - you can try them with movement, choose the option below
const unsigned int option = 5;                                                 // change this to try the options below on any axis
static int JoyStick2Speed( int nJoyX, unsigned int choice )
{
    switch (choice)
    {
      case 1u:
        return static_cast<int>(((static_cast<float>(nJoyX)) + pow((static_cast<float>(nJoyX)),2.0f))/2.0f);              // option 1
      case 2u:
        return static_cast<int>(((2.0f*(static_cast<float>(nJoyX))) + pow((static_cast<float>(nJoyX)),2.0f))/3.0f);       // option 2
      case 3u:
        return static_cast<int>(((static_cast<float>(nJoyX))+(2.0f*pow((static_cast<float>(nJoyX)),2.0f)))/3.0f);         // option 3
      case 4u:
         return static_cast<int>(pow((static_cast<float>(nJoyX)),2.0f));                                                  // option 4
      default:
         return(nJoyX);                                                         // option not specified then return it back
    }
}

static void printinfo( std::string msg ) {
	std::cout << msg << std::endl;
}

// USB Camera == Elecom Webcam [UCAM-C520FEBK]
void cameraPropertySetting(crewbo::camera::UsbCamera& camera, int b=10, int c=20, int s=36, int h=0, int g= 80) {  
    camera.setCameraProperty_(cv::CAP_PROP_BRIGHTNESS, 10);
    camera.setCameraProperty_(cv::CAP_PROP_CONTRAST, 20);
    camera.setCameraProperty_(cv::CAP_PROP_SATURATION, 36);
    camera.setCameraProperty_(cv::CAP_PROP_HUE, 0);
    camera.setCameraProperty_(cv::CAP_PROP_GAMMA, 80);
}

struct MouseParameter {
    int u;
    int v;
    bool click = false;
};

// run this callback for the mouse :: use left down button to read the rgb/hsv for that co-ordinate
void mouseCallback(int event, int x, int y, int flags, void* user_data) {
    (void)flags;
    if (event == cv::EVENT_LBUTTONDOWN) {
        MouseParameter* mouse_parameter = static_cast<MouseParameter*>(user_data);
        mouse_parameter->u = x;
        mouse_parameter->v = y;
        mouse_parameter->click = true;
    }
}

// update a control window with the data from the mouse click pixcel point.
void updateControlWindow(
        const std::string& control_window_name,
        const cv::Scalar& rgb,
        const cv::Scalar& hsv) {
			
    cv::Mat controlImage(200, 400, CV_8UC3, cv::Scalar(0, 0, 0));
    double font_scale = 0.5;        
    int thickness = 1;            
    int line_type = cv::LINE_AA;   

    cv::putText(
            controlImage,
            "RGB : " + std::to_string(static_cast<int>(rgb[0])) + "," +
                    std::to_string(static_cast<int>(rgb[1])) + "," + std::to_string(static_cast<int>(rgb[2])),
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            font_scale,
            cv::Scalar(255, 255, 255),
            thickness,
            line_type);

    cv::putText(
            controlImage,
            "HSV : " + std::to_string(static_cast<int>(hsv[0])) + "," +
                    std::to_string(static_cast<int>(hsv[1])) + "," +
                    std::to_string(static_cast<int>(hsv[2])),
            cv::Point(10, 60),
            cv::FONT_HERSHEY_SIMPLEX,
            font_scale,
            cv::Scalar(255, 255, 255),
            thickness,
            line_type);

    cv::imshow(control_window_name, controlImage);
}

int main(int argc, char** argv) {
    cv::String keys = "{brightness||}""{contrast||}""{saturation||}""{hue||}""{gamma||}""{width||}""{height||}""{ipaddress||}""{port}";
    cv::CommandLineParser parser(argc, argv, keys);
    const int image_width = static_cast<int>(parser.get<cv::int>("width"));
    const int image_height = static_cast<int>(parser.get<cv::int>("height"));
    const int bri = static_cast<int>(parser.get<cv::int>("brightness"));
    const int cont = static_cast<int>(parser.get<cv::int>("contrast"));
    const int sat = static_cast<int>(parser.get<cv::int>("saturation"));
    const int hue = static_cast<int>(parser.get<cv::int>("hue"));
    const int gam = static_cast<int>(parser.get<cv::int>("gamma"));
    const std::string ip_addr = static_cast<std::string>(parser.get<cv::string>("ipaddress"));	
    const unsigned int iport = static_cast<unsigned int>(parser.get<cv::int>("gamma"));
	
    system("start_melfa");

    // realsense and usb camera & display
    //const int image_width = 1280;
    //const int image_height = 720;
    crewbo::camera::UsbCamera camerau(0, image_width, image_height);
	cameraPropertySetting(camerau, bri, cont, sat, hue, gam);
    cv::String pic_window_name = "hsv color picker";
    cv::namedWindow(pic_window_name);
	
    crewbo::camera::RealSense camera({640, 360});
    va::Display display("realsense robot vision");

    // joystick
    const char* device_file = "/dev/input/js0";
    const int max_abs_value = 32767 - 1;
    crewbo::joystick::JoyStick JoyStick(device_file, max_abs_value);
    std::vector<int> joystick_axis;
    std::vector<int> joystick_button;

    // choose mitsubishi melfa robot RV-4FR-L or RV-5AS robot
	// const std::string ip_addr = "192.168.15.20";
	// const unsigned int iport = 10000U;
#define USING_ROBOT 3                                                                // RV-5AS with specified ip address and port
#ifdef USING_ROBOT == 0	
    crewbo::robot::melfa::base_kit::MelfaRv4frl robot;
#endif
#ifdef USING_ROBOT == 3
    crewbo::robot::melfa::base_kit::MelfaRv5as robot(1.f, ip_addr, iport);
#endif
#ifdef USING_ROBOT == 1	
    crewbo::robot::melfa::base_kit::MelfaRv4frl robot(1.f, ip_addr, iport);
#endif
#ifdef USING_ROBOT == 2
    crewbo::robot::melfa::base_kit::MelfaRv5as robot;
#endif
    using namespace crewbo::literals;

    // mouse 
    MouseParameter mouse;
    mouse.u = 0;
    mouse.v = 0;
    cv::setMouseCallback(pic_window_name, mouseCallback, &mouse);

    // window to write color values relating to the point clicked by mouse 
    cv::String ctl_window_name = "Control Window";
    int window_wt = 400;
    int window_ht = 200;
    cv::namedWindow(ctl_window_name);
    cv::resizeWindow(ctl_window_name, window_wt, window_ht);

    printinfo("You can operate the robot.Exit with the button [13].");
    printinfo("Left stick = X,Y /J2, J1");
    printinfo("Right stick = Z,C/J3, J4");
    printinfo("Cross key = A,B /J5, J6");
    printinfo ("Button 1: Slow mode, button 2: Fast mode, button 3: POSE mode, button 4: JOINT mode");	
    bool is_operatable = true;
    float override_value = 0.1f;
    bool pose_mode = true;

    while (is_operatable == true) {
        if (pose_mode == true) {
            crewbo::robot::concurrent_updater::ConcurrentPoseUpdater updater(&robot, 0.2f, 1e-3f);

            while (true) {
                const cv::Mat camera_image = camera.fetchSingleFrame_();
                display.show(camera_image);
                cv::Mat image = camerau.fetchSingleFrame_();
                cv::Mat original_image = image.clone();  
                if (mouse.u + mouse.v > 0) {
                    cv::circle(image, {mouse.u, mouse.v}, 3, {0, 0, 255}, -1);
                }
                cv::imshow(pic_window_name, image);
		
                crewbo::Pose current_pose = robot.getCurrentPose_();
                std::cout << "POSE : " << current_pose << std::endl;

                JoyStick.readValue();
                joystick_axis = JoyStick.axisValue();
                joystick_button = JoyStick.buttonValue();

                if (joystick_button[12]) {
                    is_operatable = false;
                    break;
                }
                if (joystick_button[0]) {
                    override_value = 0.1f;
                    std::cout << "override : " << override_value << std::endl;
                }
                if (joystick_button[1]) {
                    override_value = 0.5f;
                    std::cout << "override : " << override_value << std::endl;
                }
                if (joystick_button[2]) {
                    pose_mode = true;
                    break;
                }
                if (joystick_button[3]) {
                    pose_mode = false;
                    break;
                }

                const float error_x = applyDeadzone(static_cast<float>(-JoyStick2Speed(joystick_axis[1], option)));
                const float error_y = applyDeadzone(static_cast<float>(JoyStick2Speed(joystick_axis[0], option)));
                const float error_z = applyDeadzone(static_cast<float>(-JoyStick2Speed(joystick_axis[2], option)));
                const float error_ra = applyDeadzone(static_cast<float>(joystick_axis[3]));
                const float error_rb = applyDeadzone(static_cast<float>(joystick_axis[5]));
                const float error_rc = applyDeadzone(static_cast<float>(joystick_axis[4]));
                const crewbo::Degrees error_ra_degree{error_ra};
                const crewbo::Degrees error_rb_degree{error_rb};
                const crewbo::Degrees error_rc_degree{error_rc};

                const float p_gain_for_position = 0.05f * override_value;
                const float input_x = p_gain_for_position * error_x;
                const float input_y = p_gain_for_position * error_y;
                const float input_z = p_gain_for_position * error_z;

                const float p_gain_for_rotation = 0.01f * override_value;
                const crewbo::Degrees input_ra = p_gain_for_rotation * error_ra_degree;
                const crewbo::Degrees input_rb = p_gain_for_rotation * error_rb_degree;
                const crewbo::Degrees input_rc = p_gain_for_rotation * error_rc_degree;

                const crewbo::Pose input_rel_pose{input_x, input_y, input_z, input_ra, input_rb, input_rc};
                updater.updateGoalByRelPose_(input_rel_pose);

                if (mouse.click) {
                    std::cout << "clicked" << std::endl;
                    mouse.click = false;

                    cv::Vec3b rgb_pixel = original_image.at<cv::Vec3b>(mouse.v, mouse.u);
                    int r = rgb_pixel[2];
                    int g = rgb_pixel[1];
                    int b = rgb_pixel[0];
                    std::cout << "RGB: (" << r << ", " << g << ", " << b << ")" << std::endl;

                    cv::Mat rgb_mat(1, 1, CV_8UC3, cv::Scalar(b, g, r));
                    cv::Mat hsv_mat;
                    cv::cvtColor(rgb_mat, hsv_mat, cv::COLOR_BGR2HSV);

                    cv::Vec3b hsv_pixel = hsv_mat.at<cv::Vec3b>(0, 0);
                    int h = hsv_pixel[0];
                    int s = hsv_pixel[1];
                    int v = hsv_pixel[2];
					
					cv::Scalar rgb(r, g, b);
					cv::Scalar hsv(h, s, v);	
                    updateControlWindow(ctl_window_name, rgb, hsv);						
                }
			}
        } else {
            crewbo::robot::concurrent_updater::ConcurrentJointUpdater updater(&robot, 0.2f, 1e-3f);

            while (true) {
                const cv::Mat camera_image = camera.fetchSingleFrame_();
                display.show(camera_image);
                cv::Mat image = camerau.fetchSingleFrame_();
                cv::Mat original_image = image.clone();  
                if (mouse.u + mouse.v > 0) {
                    cv::circle(image, {mouse.u, mouse.v}, 3, {0, 0, 255}, -1);
                }
                cv::imshow(pic_window_name, image);
				
                crewbo::Joint current_joint = robot.getCurrentJoint_();
                std::cout << "JOINT: " << current_joint << std::endl;

                JoyStick.readValue();
                joystick_axis = JoyStick.axisValue();
                joystick_button = JoyStick.buttonValue();

                if (joystick_button[12]) {
                    is_operatable = false;
                    break;
                }
                if (joystick_button[0]) {
                    override_value = 0.1f;
                    std::cout << "override : " << override_value << std::endl;
                }
                if (joystick_button[1]) {
                    override_value = 0.5f;
                    std::cout << "override : " << override_value << std::endl;
                }
                if (joystick_button[2]) {
                    pose_mode = true;
                    break;
                }
                if (joystick_button[3]) {
                    pose_mode = false;
                    break;
                }

                const float error_j1 = applyDeadzone(static_cast<float>(joystick_axis[0]));
                const float error_j2 = applyDeadzone(static_cast<float>(joystick_axis[1]));
                const float error_j3 = applyDeadzone(static_cast<float>(joystick_axis[2]));
                const float error_j4 = applyDeadzone(static_cast<float>(joystick_axis[3]));
                const float error_j5 = applyDeadzone(static_cast<float>(joystick_axis[5]));
                const float error_j6 = applyDeadzone(static_cast<float>(joystick_axis[4]));
                const crewbo::Degrees error_j1_degree{error_j1};
                const crewbo::Degrees error_j2_degree{error_j2};
                const crewbo::Degrees error_j3_degree{error_j3};
                const crewbo::Degrees error_j4_degree{error_j4};
                const crewbo::Degrees error_j5_degree{error_j5};
                const crewbo::Degrees error_j6_degree{error_j6};

                const float p_gain_for_rotation = 0.01f * override_value;
                const crewbo::Degrees input_j1 = p_gain_for_rotation * error_j1_degree;
                const crewbo::Degrees input_j2 = p_gain_for_rotation * error_j2_degree;
                const crewbo::Degrees input_j3 = p_gain_for_rotation * error_j3_degree;
                const crewbo::Degrees input_j4 = p_gain_for_rotation * error_j4_degree;
                const crewbo::Degrees input_j5 = p_gain_for_rotation * error_j5_degree;
                const crewbo::Degrees input_j6 = p_gain_for_rotation * error_j6_degree;

                const crewbo::Joint input_rel_joint{input_j1, input_j2, input_j3, input_j4, input_j5, input_j6};
                updater.updateGoalByRel_(input_rel_joint);

                if (mouse.click) {
                    std::cout << "clicked" << std::endl;
                    mouse.click = false;

                    cv::Vec3b rgb_pixel = original_image.at<cv::Vec3b>(mouse.v, mouse.u);
                    int r = rgb_pixel[2];
                    int g = rgb_pixel[1];
                    int b = rgb_pixel[0];
                    std::cout << "RGB: (" << r << ", " << g << ", " << b << ")" << std::endl;

                    cv::Mat rgb_mat(1, 1, CV_8UC3, cv::Scalar(b, g, r));
                    cv::Mat hsv_mat;
                    cv::cvtColor(rgb_mat, hsv_mat, cv::COLOR_BGR2HSV);

                    cv::Vec3b hsv_pixel = hsv_mat.at<cv::Vec3b>(0, 0);
                    int h = hsv_pixel[0];
                    int s = hsv_pixel[1];
                    int v = hsv_pixel[2];

					cv::Scalar rgb(r, g, b);
					cv::Scalar hsv(h, s, v);
                    updateControlWindow(ctl_window_name, rgb, hsv);					
                }
            }
        }
    }
	cv::destroyAllWindows();
	return 0;
}