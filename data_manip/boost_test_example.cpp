//
// Boost TEST Example to test sort algorythm
//
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <string>

#include <cstdlib>
#include <ctime>

#include <utility>
#include <random>
#include <iomanip>

#include <algorithm>
#include <exception>
#include <iterator>
#include <typeinfo>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

namespace{
using namespace std;
using namespace boost;

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

// bubble sort adapted for double fields
void doubleBubbleSort(vector<double>::iterator first, vector<double>::iterator last, int show_res) {
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
      std::cout << std::fixed << std::setprecision(8) << *e << ' ';
    }
    std::cout << std::endl;
  }
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

unsigned int solve_sort(unsigned int choice_col) {
  constexpr int num_data_items = 60;
  data_t myData[num_data_items];
  std::vector<data_t> ss;
  std::vector<int> time_list;
  double max_v;
  double min_v;
  unsigned int test_result = 1;
  
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

  // print out the sorted result and check if we sorted it correctly
  double prev_val = -200000;                                                                                                    // the most negative depth that can be seen  
  for (auto i : out) {
    std::cout << std::fixed << std::setprecision(1) << i.a_val << " " << std::fixed << std::setprecision(3) << i.b_val << " " << i.c_val << " " << std::scientific << std::setprecision(13) << i.t_val << std::endl;
    auto cv = get_value(i,choice_col);
    if (cv < prev_val) {                                                                                                        // fail test
        test_result = 0;
		break;
    }
	prev_val = cv;
  }
  return test_result; 
 
}

unsigned int solve_bubble_sort(unsigned int choice_col) {
  constexpr int num_data_items = 60;
  data_t myData[num_data_items];
  std::vector<data_t> ss;
  std::vector<int> time_list;
  std::vector<double> dval_list;
  double max_v;
  double min_v;
  unsigned int test_result = 1;
  
  // populate the data structures with input data (random) and create the input list for sorting
  for (int ii = 0; ii < num_data_items; ++ii) {
	  myData[ii].a_val = static_cast<double>(rand_gen_num(0,100,(ii%10),((ii*2)%2)));
	  myData[ii].b_val = static_cast<double>(rand_gen_num(0,100,((ii*3)%10),((ii*3)%2)));
	  myData[ii].c_val = rand_gen_num(-10.0,10.0,1,1);	
      myData[ii].t_val = static_cast<int>((time(NULL)- rand_gen_num(-3,3,0,0)));
      ss.push_back(myData[ii]);	 
      switch (choice_col) {
        case 0:
        dval_list.push_back(myData[ii].a_val);
		break;		
        case 1:
        dval_list.push_back(myData[ii].b_val);
		break;
        case 2:
        dval_list.push_back(myData[ii].c_val);
		break;
        case 3:	  
        time_list.push_back(myData[ii].t_val);
		break;
      } 

      std::cout << myData[ii].t_val << std::endl;
  }	
  std::vector<data_t> out = {};                                           // create a vector of data objects in sorted order
  ss.reserve(20000);                                                      // reserver the memory for speed
  out.reserve(20000); 
      
  // ------------- bubble sort the times in the time_list vector and create the sorted output list -----------------
  std::cout << "[BUBBLE SORT]" << std::endl;
  switch (choice_col) {
        case 0:
        doubleBubbleSort(dval_list.begin(), dval_list.end(), 0);
		break;		
        case 1:
        doubleBubbleSort(dval_list.begin(), dval_list.end(), 0);
		break;
        case 2:
        doubleBubbleSort(dval_list.begin(), dval_list.end(), 0);
		break;
        case 3:	  
        bubbleSort(time_list.begin(), time_list.end(), 0);
		break;
  } 
  
  int cur_v = -9000;
  std::vector<data_t> tt;
  if (choice_col == 3) {
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
  } else {
    for (auto tim_v : dval_list) {
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
  }

  double prev_val = -200000;                                                                                                    // the most negative depth that can be seen  
  for (auto i : tt) {
    std::cout << std::fixed << std::setprecision(1) << i.a_val << " " << std::fixed << std::setprecision(3) << i.b_val << " " << i.c_val << " " << std::scientific << std::setprecision(13) << i.t_val << std::endl;
    auto cv = get_value(i,choice_col);
    if (cv < prev_val) {                                                                                                        // fail test
        test_result = 0;
		break;
    }
	prev_val = cv;
  } 
  return test_result;     
 
}

auto total_col(vector<int>::iterator first, vector<int>::iterator last) {
    auto a = std::accumulate(first, last, 0);
    int msb = 0;
    if (a < 0) { ++msb; }
    return std::make_pair(static_cast<unsigned int>(a), msb);
}
auto double_total_col(vector<double>::iterator first, vector<double>::iterator last) {
    auto a = std::accumulate(first, last, 0);
    return static_cast<double>(a);
}

bool solve_sum_col(unsigned int choice_col) {
  constexpr int num_data_items = 60;
  data_t myData[num_data_items];
  std::vector<data_t> ss;
  std::vector<int> time_list;
  std::vector<double> dval_list;
  double max_v;
  double min_v;
  unsigned int test_result = 1;
  
  // populate the data structures with input data (random) and create the input list for sorting
  for (int ii = 0; ii < num_data_items; ++ii) {
	  myData[ii].a_val = static_cast<double>(rand_gen_num(0,100,(ii%10),((ii*2)%2)));
	  myData[ii].b_val = static_cast<double>(rand_gen_num(0,100,((ii*3)%10),((ii*3)%2)));
	  myData[ii].c_val = rand_gen_num(-10.0,10.0,1,1);	
      myData[ii].t_val = static_cast<int>((time(NULL)- rand_gen_num(-3,3,0,0)));
      ss.push_back(myData[ii]);	 
      switch (choice_col) {
        case 0:
        dval_list.push_back(myData[ii].a_val);
		break;		
        case 1:
        dval_list.push_back(myData[ii].b_val);
		break;
        case 2:
        dval_list.push_back(myData[ii].c_val);
		break;
        case 3:	  
        time_list.push_back(myData[ii].t_val);
		break;
      } 

      std::cout << myData[ii].t_val << std::endl;
  }	
  std::vector<data_t> out = {};                                           // create a vector of data objects in sorted order
  ss.reserve(20000);                                                      // reserver the memory for speed
  out.reserve(20000); 
      
  // ------------- bubble sort the times in the time_list vector and create the sorted output list -----------------
  std::cout << "[BUBBLE SORT]" << std::endl;
  double dcol = 0.0;
  std::pair<unsigned int, int>  icol;
  
  switch (choice_col) {
        case 0:
        dcol = double_total_col(dval_list.begin(), dval_list.end(), 0);
		break;		
        case 1:
        dcol = double_total_col(dval_list.begin(), dval_list.end(), 0);
		break;
        case 2:
        dcol = double_total_col(dval_list.begin(), dval_list.end(), 0);
		break;
        case 3:	  
        icol = total_col(time_list.begin(), time_list.end(), 0);
		break;
  } 

  if (choice_col == 3) {
      unsigned int dd = 0;
      unsigned int ddlast = 0;
      int roll = 0;
      for (v : time_list) {
	      dd += v;
	      if (dd < ddlast) ++roll;
          ddlast = dd;
      }
	  if (roll != 0) { std::cout << "rollover occurred" << std::cout; }
      return (dd == icol.first);
  } else { 
      double dd = 0.0;
      for (v : dval_list) {
	      dd += v;
      }
      return (dd == dcol);
  }
}

}

// =============================== unit tests ===================================================
//
BOOST_AUTO_TEST_CASE( solve_test1 ){
    BOOST_CHECK_EQUAL(solve_sort(0), 1);
}

BOOST_AUTO_TEST_CASE( solve_test2 ){
    BOOST_CHECK_EQUAL(solve_sort(1), 1);
}

BOOST_AUTO_TEST_CASE( solve_test3 ){
    BOOST_CHECK_EQUAL(solve_sort(2), 1);
}

BOOST_AUTO_TEST_CASE( solve_test4 ){
    BOOST_CHECK_EQUAL(solve_sort(3), 1);
}

BOOST_AUTO_TEST_CASE( solve_test5 ){
    BOOST_CHECK_EQUAL(solve_bubble_sort(0), 1);
}

BOOST_AUTO_TEST_CASE( solve_test6 ){
    BOOST_CHECK_EQUAL(solve_bubble_sort(1), 1);
}

BOOST_AUTO_TEST_CASE( solve_test7 ){
    BOOST_CHECK_EQUAL(solve_bubble_sort(2), 1);
}

BOOST_AUTO_TEST_CASE( solve_test8 ){
    BOOST_CHECK_EQUAL(solve_bubble_sort(3), 1);
}

BOOST_AUTO_TEST_CASE( solve_test9 ){
    BOOST_CHECK_EQUAL(solve_sum_col(0), true);
}

BOOST_AUTO_TEST_CASE( solve_test10 ){
    BOOST_CHECK_EQUAL(solve_sum_col(1), true);
}

BOOST_AUTO_TEST_CASE( solve_test11 ){
    BOOST_CHECK_EQUAL(solve_sum_col(2), true);
}

BOOST_AUTO_TEST_CASE( solve_test12 ){
    BOOST_CHECK_EQUAL(solve_sum_col(3), true);
}
