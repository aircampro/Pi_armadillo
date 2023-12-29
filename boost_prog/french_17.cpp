#include <algorithm>
#include <iomanip>
#include <iostream>
#include <list>
#include <numeric>
#include <random>
#include <vector>
#include <iterator>

using namespace std;
vector<int> get_test_indexs( int num_of_enums, int num_of_samples) {
//using namespace std;
vector<int> data(num_of_enums);
iota(begin(data), end(data), 1);
copy(cbegin(data), cend(data), ostream_iterator<int>(cout, " "));
cout << '\n';

random_device seeder;
const auto seed = seeder.entropy() ? seeder() : time(nullptr);
default_random_engine generator(
       static_cast<default_random_engine::result_type>(seed));

const size_t numberOfSamples = num_of_samples;
vector<int> sampledData(numberOfSamples);

for (size_t i = 0; i < 10; ++i)
{
    sample(cbegin(data), cend(data), begin(sampledData),
           numberOfSamples, generator);
    copy(cbegin(sampledData), cend(sampledData),
         ostream_iterator<int>(cout, " "));
    copy(rbegin(sampledData), rend(sampledData),
         ostream_iterator<int>(cout, " "));
    cout << '\n';
}
return sampledData;	

}

int main(void) {
  vector<int> v;
  v = get_test_indexs( 32, 5 );
  // write this either way both will print to the std::cout
  for (auto i : v) std::cout << " val " << i << std::endl;
  copy(cbegin(v), cend(v), ostream_iterator<int>(cout, " val "));
}
