#include <iostream>
#include <fstream>

int main(void)
{
    std::ofstream logFile("myapp.log");
    if (logFile) {
        //std::clog.rdbuf(logFile.rdbuf());
    }
    //std::clog << "Hello, clog world!" << std::endl;
    return 0;
}
