// read csv lines and sort on the time column specified
//
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <string>

// fast vector copy
// std::copy(v2.begin(),v2.end(),std::back_inserter(v1));

// function to split the csv lines into string fields
//
std::vector<std::string> split(std::string &input, char delimiter) {
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) result.push_back(field);
    return result;
}

// function to read the lines from a file
//
std::vector<std::string> ReadToStrVec(std::string filename)
{
	std::ifstream file;
	file.open(filename);
	std::vector<std::string> contents = {};
	if (file.fail())
	{
	    std::cout << "cant open " << filename << " exiting" << std::endl;
	    return contents;
	}

	std::string line;
	while (!file.eof())
	{
	    std::getline(file, line);
	    contents.push_back(line);
	    if (file.eof()) { std::cout << "completed file read okay" << std::endl; }
	}

	return contents;
}

int main() {

  // read the data from the input file
  std::vector<std::string> ss = ReadToStrVec("input_data.csv");
  if (ss.size() == 0) {
      std::quick_exit(1);
  }
  // create a vector of test data (if youdont have the input file)
  //std::vector<std::string> ss = {"90,0.3,74,43","1,0.3,34,3","1,2.3,34,3","1,7.3,34,3","97,0.3,94,03","1,8.13,34,3","1,8.15,34,53","1,8.05,64,13","1,8.18,74,13","501,8.07,74,13","13,0.37,34,3"};
  std::vector<std::string> out = {};
  ss.reserve(20000);                                                      // reserver the memory for speed
  out.reserve(20000); 
  
  unsigned int line_no = 1;
  const unsigned int time_col = 1;                                         // define the sort column as 1 (starts @ 0)
 
  for ( auto line : ss ) {
    if (line_no == 1) {                                                   // first line
       out.push_back(line);
       ++line_no;
    } else if (line_no == 2) {                                            // second line
       std::vector<std::string> row_list = split(line,',');
       auto time_spec_line = std::stod(row_list.at(time_col));
       std::vector<std::string> out_list = split(out.at(0),',');
       auto time_spec_out = std::stod(out_list.at(time_col));
       if (time_spec_line > time_spec_out) {
           out.push_back(line);
       } else {
           auto it2 = out.begin(); 
           out.insert(it2, line);
       }
       ++line_no;
    } else {                                                               // next lines starting @ line 3
       std::vector<std::string> row_list = split(line,',');
       auto time_spec_line = std::stod(row_list.at(time_col));
       std::vector<std::string> out_list = split(out.at(0),',');
       auto time_spec_out_front = std::stod(out_list.at(time_col));
       out_list = split(out.at(out.size()-1),',');
	   auto time_spec_out_back = std::stod(out_list.at(time_col));
	   if ((time_spec_line > time_spec_out_front) && (time_spec_line < time_spec_out_back)) {
           auto d1 = time_spec_line - time_spec_out_front;
           auto d2 = time_spec_out_back - time_spec_line;
           if (d1 < d2) {                                                     // closest to the front time
               auto it3 = out.begin();                                        // point to the strt of the vector list
	       ++it3;                                                         // advance 1 record fprward as we already checked it		   
               auto length = out.size();
               for (auto i = 1; i < static_cast<int>(length); ++i) {                            // advance forward through the output records
                   out_list = split(out.at(i),',');
	           time_spec_out_front = std::stod(out_list.at(time_col));
                   if (time_spec_line < time_spec_out_front) {
			out.insert(it3, line);
			break;
                   }
                   ++it3;				   
               }		   
           } else {	                                                         // time read is closer to the back of the out list
               auto length = out.size();
               auto it4 = out.end();                                         // point to the end of the vector
	       --it4;                                                        // advance 1 record back already done it
               for (auto i = length-2; i > 0; --i) {                         // advance backward through the output records
                   out_list = split(out.at(i),',');
	           time_spec_out_back = std::stod(out_list.at(time_col));
                   if (time_spec_line > time_spec_out_back) {
			out.insert(it4, line);
			break;
                   }
                   --it4;				   
               }
           }
       } else if (time_spec_line <= time_spec_out_front) {
           auto it5 = out.begin(); 
           out.insert(it5, line);
       } else if (time_spec_line >= time_spec_out_back) {
           out.push_back(line);
       }
	++line_no;	
    }
  }

  // print the output lines sorted numerically in time and write them to a file
  std::ofstream writing_file;
  writing_file.open("test2out.csv");
  if (writing_file.fail()) {
     std::cout << "write file failed to open" << std::endl;
     std::quick_exit(2);
  }
  for (auto i : out) {
    std::cout << i << std::endl;
    writing_file << i;
    writing_file << std::endl;
  }
  return 0;	
}
