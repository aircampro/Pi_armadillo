#ifndef _joys_
#define _joys_

#include <iostream>
#include <vector>

namespace crewbo::joystick {
class JoyStick {
public:
    JoyStick(const char* device_file, const int max_abs_value = 1);
    ~JoyStick();
    void readValue();
    std::vector<int> axisValue();
    std::vector<int> buttonValue();

private:
    int joy_fd;
    int num_of_axis;
    int num_of_buttons;
    float max_abs_value_for_percentage;
    char name_of_joystick[80];
    std::vector<int> joy_axis;
    std::vector<int> joy_button;
};

}  // namespace crewbo::joystick
#endif