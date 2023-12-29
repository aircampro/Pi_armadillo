#include <iostream>
#include <algorithm>

int main(void) {
int b = std::clamp(3, 1, 6); 
int a = std::clamp(0, 1, 6);  
int c = std::clamp(8, 1, 6);

std::cout << b << a << c << "\n";
}
