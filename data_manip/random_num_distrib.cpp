// lib for other distributions of random data 
// each one is using the mt19937 generator function
//
#include <iostream>
#include <random>
#include <set>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <algorithm>

// bernoulli
auto create_bool_map(int num_val) {
    std::random_device rd;
    std::mt19937 gen(rd());
    // give "true" 1/4 of the time
    // give "false" 3/4 of the time
    std::bernoulli_distribution d(0.25);
 
    std::map<bool, int> hist;
    for (int n = 0; n < num_val; ++n)
        ++hist[d(gen)];
    return hist;
}
		
// poisson_distribution
// nrolls = 10000; // number of experiments
// nstars = 100;   // maximum number of stars to distribute
//
auto create_poisson_distribution(int nrolls, int nstars)
{
  std::default_random_engine generator;
  std::poisson_distribution<int> distribution(4.1);

  int p[10]={};
  std::set<int> set_v = {};
  std::vector<int> v_v = {};
  
  for (int i=0; i<nrolls; ++i) {
    int number = distribution(generator);
    if (number<10) {
	  ++p[number];
	}
  }
  for (unsigned int z = 0; z < 10; ++z) {
    set_v.insert(p[z]);
	v_v.push_back(p[z]);
  }
  return std::make_pair(set_v, v_v);
}

int create_weibull_distribution(int num_val)
{
    std::random_device rd;
    std::mt19937 gen(rd());
 
    std::weibull_distribution<> d;
 
    std::map<int, int> hist;
    for (int n = 0; n != num_val; ++n)
        ++hist[std::round(d(gen))];
 
    returb hist;
}

// cauchy_distribution
// nrolls=10000;   number of experiments
// nstars=100;     maximum number of stars to distribute
//
auto create_cauchy_distribution(int nrolls, int nstars)
{

  std::default_random_engine generator;
  std::cauchy_distribution<double> distribution(5.0,1.0);

  int p[10]={};
  std::set<int> set_v = {};
  std::vector<int> v_v = {};
  
  for (int i=0; i<nrolls; ++i) {
    double number = distribution(generator);
    if ((number>=0.0)&&(number<10.0)) ++p[int(number)];
  }

  for (unsigned int z = 0; z < 10; ++z) {
    set_v.insert(p[z]);
	v_v.push_back(p[z]);
  }
  return std::make_pair(set_v, v_v);
}


// find position of element in set
auto find_val_in_set(std::set<int>& st, int v2f) {
	auto first = st.find(v2f);
	return first;
}

// delete a value from the set
auto del_val_in_set(std::set<int>& st, int v2f) {
	auto first = st.find(v2f);
    auto b = st.erase(first);
	return *b;                                       // return next element value
}

// fast vector copy
// std::copy(v2.begin(),v2.end(),std::back_inserter(v1));

auto del_from_vals_in_set(std::set<int>& st, int v_st, int v_end) {
	auto first = st.find(v_st);
	auto last = st.find(v_end);
	st.erase(first, last);
	//for (auto itr = st.begin(); itr != st.end(); ++itr)
	//	std::cout << *itr;
    return first;
}

// leave odd numbers only
void remove_even_set(std::set<int>& st) {
	for (auto itr = st.begin(); itr != st.end();) {
		if (*itr % 2 == 0)
			itr = st.erase(itr);
		else
			++itr;;
	}
}

// leave even numbers only
auto remove_odd_set(std::set<int>& st) {
	for (auto itr = st.begin(); itr != st.end();) {
		if (!static_cast<bool>((*itr % 2)) == false)
			itr = st.erase(itr);
		else
			++itr;;
	}
}

auto shuffle_vector(std::vector<int>& v, int start_s, int end_s) {
    auto const seed = std::random_device{}();
    auto reng = std::mt19937{seed};
    shuffle(begin(v)+start_s, begin(v)+end_s, reng);  
    return shuffle;
}

auto create_fisher_f_distribution(int a, int b)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::fisher_f_distribution<double> dist(a, b);
    std::set<double> set_v = {};
    std::vector<double> v_v = {};
    for (unsigned int f=0; f<numvals ; ++f) {
        set_v.insert(dist(gen));
        v_v.emplace_back(dist(gen));
    }
    return std::make_pair(set_v, v_v);
}


