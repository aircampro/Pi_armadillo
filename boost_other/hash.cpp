    #include <iostream>
  #include <string>
  #include <unordered_map>
  
  #include "absl/hash/hash.h"

  int main() {
	  	std::unordered_map<std::string, int, absl::Hash<std::string>> my_map = {{"one",    1}, {"two", 2}, {"three", 3}};
			
			for(auto i : my_map) {
						std::cout << i.first << " " << i.second << " " << std::endl;
							}
				return 0;
				   
