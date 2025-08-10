#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <string>

#include <cstdlib>
#include <ctime>

# include <utility>
# include <random>
#include <iomanip>

using namespace std;

// this is a structure containing your data
struct data_t {
    double a_val;
    double b_val;
    double c_val;
    int t_val;
};

// random number generator
double rand_gen_num(int minn, int maxn, int method, int d_choi) {
  random_device seed;   
  switch(method) {
      case 0:	{  
        mt19937 mt(seed());
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;
	  
      case 1:	{  
        minstd_rand0 mt(seed());
         switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;    	  
	  case 2:   {
        minstd_rand mt(seed());
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;	  
	  case 3:   {
        mt19937_64 mt(seed());
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;	  
	  case 4:   {
        ranlux24_base mt(seed());
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;  
      case 5:   {
        ranlux48_base mt(seed());
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;	  
	  case 6:   {
        ranlux24 mt(seed());
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;	  
	  case 7:   {
        ranlux48 mt(seed());
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;	  
	  case 8:   {
        knuth_b mt(seed()); 
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;
      case 9 :	{  
        mt19937 mt(static_cast<unsigned int>(time(nullptr)));           // time(NULL);
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;
      default :	{  
        mt19937 mt(static_cast<unsigned int>(time(nullptr)));           // time(NULL);
        switch(d_choi) {
          case 0: {
	        uniform_int_distribution<int> rnd_val(minn, maxn);
            return static_cast<double>(rnd_val(mt));
          }
          break;		  
          case 1:	{  
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
 
          default:	{ 
            uniform_real_distribution<double> rnd_val(minn, maxn);
            return rnd_val(mt);
          }
          break;
        }
      }
	  break;
  }
  
}

// gets a specific value from the structure object
double get_value(data_t& tv, unsigned int choice) {
   double ret;
   switch(choice) {
	   case 0:
	   ret = tv.a_val;
	   break;
	   case 1:
	   ret = tv.b_val;
	   break;
	   case 2:
	   ret = tv.c_val;
	   break;
	   case 3:
	   ret = tv.t_val;
	   break;
	   default:
	   ret = tv.c_val;
	   break;	   
   }
   return ret;
}

// bubble sort
void bubbleSort(vector<int>::iterator first, vector<int>::iterator last, int show_res) {
  //int count = 0;
  for(auto e = first; e != last; ++e) {
    for(auto i = last - 1; i != e; --i) {
      auto j = i;
      advance(j, -1);
      if(*i < *j) {
        iter_swap(i, j);
      }
    }
    //std::cout << "(" << count + 1 << "count " << ")" << std::endl;
    //++count;
  }
  if (show_res == 1) {
    for(auto e = first; e != last; ++e) {
      std::cout << *e << ' ';
    }
    std::cout << std::endl;
  }
}

auto total_col(vector<int>::iterator first, vector<int>::iterator last) {
    auto a = std::accumulate(first, last, 0);
    int rollo = 0;
    if (a < 0) { ++rollo; }
    return std::make_pair(static_cast<unsigned int>(a),rollo);;
}

void showArray(vector<int>::const_iterator first, vector<int>::const_iterator last) {
  for(auto e = first; e != last; ++e) {
    std::cout << *e << ' ';
  }
  std::cout << std::endl;
}

void initArray(vector<int>::iterator first, vector<int>::iterator last, int max, int min) {
  mt19937 mt(static_cast<unsigned int>(time(nullptr)));      
  uniform_int_distribution<> rnd_val(min, max);              

  for(auto e = first; e != last; ++e) {
    *e = min < max ? rnd_val(mt) : mt();
  }
}

int main() {
  constexpr int num_data_items = 60;
  data_t myData[num_data_items];
  std::vector<data_t> ss;
  std::vector<int> time_list;
  double max_v;
  double min_v;
    
  // populate the data structures with input data (random) and create the input list for sorting
  for (int ii = 0; ii < num_data_items; ++ii) {
	  myData[ii].a_val = static_cast<double>(rand_gen_num(0,100,(ii%10),((ii*2)%2)));
	  myData[ii].b_val = static_cast<double>(rand_gen_num(0,100,((ii*3)%10),((ii*3)%2)));
	  myData[ii].c_val = rand_gen_num(-10.0,10.0,1,1);	
      myData[ii].t_val = static_cast<int>((time(NULL)- rand_gen_num(-3,3,0,0)));
      ss.push_back(myData[ii]);	  
      time_list.push_back(myData[ii].t_val);
      std::cout << myData[ii].t_val << std::endl;
  }	
  std::vector<data_t> out = {};                                           // create a vector of data objects in sorted order
  ss.reserve(20000);                                                      // reserver the memory for speed
  out.reserve(20000); 

  // --------- now sort the data generated in the column specified  --------------------
  std::cout << "[COLUMN SORT]" << std::endl;
  unsigned int line_no = 1;
  const unsigned int choice_col = 3;                                      // define the sort column as 1 (starts @ 0)
 
  for ( auto line : ss ) {
    if (line_no == 1) {                                                   // first line
       out.push_back(line);
	   ++line_no;
    } else if (line_no == 2) {                                            // second line
	   auto time_spec_line = get_value(line,choice_col);
	   auto time_spec_out = get_value(out.at(0),choice_col);
	   if (time_spec_line > time_spec_out) {
           out.push_back(line);
           max_v = time_spec_line;
           min_v = time_spec_out;
       } else {
           auto it2 = out.begin(); 
           out.insert(it2, line);
           min_v = time_spec_line;
           max_v = time_spec_out;
       }
	   ++line_no;
    } else {                                                               // next lines starting @ line 3
	   auto time_spec_line = get_value(line,choice_col);
	   auto time_spec_out_front = get_value(out.at(0),choice_col);
	   auto time_spec_out_back = get_value(out.at(out.size()-1),choice_col);
       if (time_spec_line > max_v) {
           max_v = time_spec_line;
       } 
       if (time_spec_line < min_v) {
           min_v = time_spec_line;
       } 
	   if ((time_spec_line > time_spec_out_front) && (time_spec_line < time_spec_out_back)) {
           auto d1 = time_spec_line - time_spec_out_front;
           auto d2 = time_spec_out_back - time_spec_line;
           if (d1 < d2) {                                                     // closest to the front time
               auto it3 = out.begin();                                        // point to the strt of the vector list
	           ++it3;                                                         // advance 1 record fprward as we already checked it		   
               auto length = out.size();
               for (auto i = 1; i < static_cast<int>(length); ++i) {                            // advance forward through the output records
	               time_spec_out_front = get_value(out.at(i),choice_col);
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
	               time_spec_out_back = get_value(out.at(i),choice_col);
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

  // print out the sorted result
  for (auto i : out) {
    std::cout << std::fixed << std::setprecision(1) << i.a_val << " " << std::fixed << std::setprecision(3) << i.b_val << " " << i.c_val << " " << std::scientific << std::setprecision(13) << i.t_val << std::endl;
  }
  std::cout << "max = " << max_v << " min = " << min_v << std::endl;
      
  // ------------- bubble sort the times in the time_list vector and create the sorted output list -----------------
  std::cout << "[BUBBLE SORT]" << std::endl;
  bubbleSort(time_list.begin(), time_list.end(), 0);
  int cur_v = -9000;
  std::vector<data_t> tt;
  for (auto tim_v : time_list) {
    if (tim_v != cur_v) {
        for (auto line : ss) {
            auto time_spec_line = get_value(line,choice_col);
            if ( tim_v == time_spec_line) {
                tt.push_back(line);     
            }  
        }
    }
    cur_v = tim_v;
  }
  for (auto i : tt) {
    std::cout << std::fixed << std::setprecision(1) << i.a_val << " " << std::fixed << std::setprecision(3) << i.b_val << " " << i.c_val << " " << std::scientific << std::setprecision(13) << i.t_val << std::endl;
  }

  // calculate column total
  std::cout << "total : ";
  auto p = total_col(time_list.begin(), time_list.end());
  std::cout << p.first << " " << p.second;
  std::cout << std::endl;
      
  // just some more vector stuff for use
  std::vector<int> v = {3, 90, 4, 5, 2};
  std::vector<int> vs = {3, 90, 4, 5, 12};
  vector<int> vec(10, 0);
  initArray(vec.begin(), vec.end(), 99, 0);    
  showArray(vec.begin(), vec.end());
  {
    decltype(v)::iterator it = std::prev(vec.end()); 
    std::cout << "end of vector " << *it << " " << std::endl;
  }//2
  {
    decltype(v)::iterator it = std::prev(v.end(), 3); 
    std::cout << "two from end of vector " << *it << std::endl;
  }
  {
    decltype(v)::iterator it = v.begin()+1; 
    std::cout << "2nd from start of vector " << *it << std::endl;
  }   

  return 0;    
 
}
