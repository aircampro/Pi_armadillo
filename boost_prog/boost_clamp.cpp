#include <iostream>
#include <vector>
#include <boost/algorithm/clamp.hpp>

#define HI_CLAMP 9
#define LO_CLAMP 1

int main(void)
{
    using boost::algorithm::clamp;

    // clamp a vector of values in a range 
    std::vector<int> v = {-1, 2, 5, 8, 11};
    boost::algorithm::clamp_range(v, v.begin(), LO_CLAMP, HI_CLAMP);
    // print the clamped vector
    int z = 0;
    for (int x : v ) {
        ++z;
        std::cout << "vector element [" << z << "] " << x << std::endl;
    }

    // now do the same for single values 
    int x = 11;
    x = clamp(x, LO_CLAMP, HI_CLAMP);

    int y = -1;
    y = clamp(y, LO_CLAMP, HI_CLAMP);

    std::cout << "clamped up " << x << std::endl;
    std::cout << "clamped down " << y << std::endl;
}
