#include <iostream>
#include <boost/format.hpp>
#include <string>
#include <sstream>

int main(void) {
  int age = 12;
  std::string job = "programmer";
  double exposure[5] = {12.03,0.3,0.2,0.03,0.00042};
  int gain[5] = { 1, 3, 5,6, 7};
  std::string s = "hello";
  std::stringstream ss;

  ss << boost::format("{ \"enable_man_exposure\" : False, \"exposure1\" : %1%, \"exposure2\" : %2%, \"exposure3\" : %3%, \"exposure4\" : %4%, \"exposure5\" : %5%, \"gain1\" : %6%, \"gain2\" : %7%, \"gain3\" : %8%, \"gain4\" : %9%, \"gain5\" : %10%,}") % exposure[0] % exposure[1] % exposure[2] % exposure[3] % exposure[4] % gain[0] % gain[1] % gain[2] % gain[3] % gain[4];

//ss << boost::format("{ enable_man_exposure : False,  \"exposure1\" : %3%, \"gain1\" : %1%, \"gain2\" : %2% }") % gain[1] % gain[2] % exposure[3];			
//ss << boost::format("{ enable_man_exposure : False, exposure1 : %|.4|%,exposure2 : %|.4|%, exposure3 : %|.4|%, exposure4 : %|.4|%, exposure5 : %|.4|%, gain1 : %1, gain2 : %1, gain3 : %1, gain4 : %1,gain5 : %1,}") % exposure[0] % exposure[1] % exposure[2] % exposure[3] % exposure[4] % gain[0] % gain[1] % gain[2] % gain[3] % gain[4];			

  //ss << boost::format("my age:%1%njob:%2%") % age % job; 
  std::string s1 = ss.str();
  std::cout << boost::format("my age:%1%njob:%2%") % age % job << std::endl; 

  std::cout << s << std::endl;
  std::cout << s1 << std::endl;
  return 0;
}
