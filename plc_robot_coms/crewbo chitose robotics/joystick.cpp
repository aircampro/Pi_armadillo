#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iomanip>
#include <iostream>

#include "project_lib/joystick.h"

namespace crewbo::joystick {

JoyStick::JoyStick(const char* device_file, const int max_abs_value) {
    if ((this->joy_fd = open(device_file, O_RDONLY)) < 0) {
        std::cerr << "Error in crewbo::joystick::JoyStick::JoyStick(): デバイスをオープンできません。" << device_file;
        return;
    }

    this->num_of_axis = 0;
    this->num_of_buttons = 0;
    if (max_abs_value > 1) {
        this->max_abs_value_for_percentage = static_cast<float>(max_abs_value) / 100.f;
    } else {
        this->max_abs_value_for_percentage = 1;
    }

    ioctl(this->joy_fd, JSIOCGAXES, &this->num_of_axis);
    ioctl(this->joy_fd, JSIOCGBUTTONS, &this->num_of_buttons);
    ioctl(this->joy_fd, JSIOCGNAME(80), &this->name_of_joystick);

    this->joy_button.resize(num_of_buttons, 0);
    this->joy_axis.resize(num_of_axis, 0);

    std::cout << "Joystick: " << this->name_of_joystick << std::endl;
    std::cout << this->num_of_axis << " - axis, " << this->num_of_buttons << " - buttons" << std::endl;
    fcntl(joy_fd, F_SETFL, O_NONBLOCK);  // using non-blocking mode
}

JoyStick::~JoyStick() { close(this->joy_fd); }

void JoyStick::readValue() {
    js_event js;

    read(this->joy_fd, &js, sizeof(js_event));

    switch (js.type & ~JS_EVENT_INIT) {
        case JS_EVENT_AXIS:
            if ((int)js.number >= this->joy_axis.size()) {
                std::cerr << "err:" << (int)js.number << std::endl;
            }
            this->joy_axis[(int)js.number] =
                    static_cast<int>(static_cast<float>(js.value) / this->max_abs_value_for_percentage);
            break;
        case JS_EVENT_BUTTON:
            if ((int)js.number >= this->joy_button.size()) {
                std::cerr << "err:" << (int)js.number << std::endl;
            }
            this->joy_button[(int)js.number] = js.value;
            break;
        default:
            break;
    }

    usleep(100);
}

std::vector<int> JoyStick::axisValue() { return this->joy_axis; }

std::vector<int> JoyStick::buttonValue() { return this->joy_button; }

}  // namespace crewbo::joystick