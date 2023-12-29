#include <boost/algorithm/string.hpp>
#include <boost/variant.hpp>
#include <string>
#include <iostream>

using namespace boost::algorithm;

struct show : public boost::static_visitor<>
{
  template <typename T>
  void operator()(T t) const { std::cout << t << '\n'; }
};

#define print_anything boost::apply_visitor

int main()
{
        std::string s = "Cpp Secrets.Coms";
	std::cout << s << '\n';  
	std::cout << erase_first_copy(s, "s") << '\n';
	std::cout << erase_nth_copy(s, "s", 1) << '\n';
	std::cout << erase_last_copy(s, "s") << '\n';
	std::cout << erase_all_copy(s, "s") << '\n';
	std::cout << erase_head_copy(s, 5) << '\n';
	std::cout << erase_tail_copy(s, 9) << '\n';
	std::cout.setf(std::ios::boolalpha);
	std::cout << starts_with(s, "Cpp") << '\n';
	std::cout << ends_with(s, ".com") << '\n';
	std::cout << contains(s, "Secrets") << '\n';
	std::cout << lexicographical_compare(s, "Cppz") << '\n';
	std::cout << starts_with(s, "C++") << '\n';
	std::cout << ends_with(s, "S") << '\n';
	std::cout << contains(s, "boost") << '\n';
	std::cout << lexicographical_compare(s, "Cpp") << '\n'; 
	show s2;
	boost::variant<int, double, char, std::string> v;
	v = 82.4;
	boost::apply_visitor(s2, v);
	std::cout << boost::get<double>(v) << std::endl;
	v = 'C';
	boost::apply_visitor(s2, v);
	v = "myBoost::variant";
	boost::apply_visitor(s2, v);
	std::cout << boost::get<std::string>(v) << std::endl;
	auto t1 = boost::get<std::string>(v);
        std::cout << t1 << std::endl;
	v = contains(s, "p S");
	boost::apply_visitor(s2, v);
	std::string T = "--Cpp Secrets. Com--";
	std::cout << trim_left_copy_if(T, is_any_of("-")) << '\n';
	std::cout << trim_right_copy_if(T, is_any_of("-")) << '\n';
	std::cout << trim_copy_if(T, is_any_of("-")) << '\n';
	std::cout << "Original String --" << s << '\n';  
	std::cout << replace_first_copy(s, "Cpp", "C plus plus") << '\n';
	std::cout << replace_nth_copy(s, "C", 0, "S") << '\n';
	std::cout << replace_last_copy(s, "C", "S") << '\n';
	std::cout << replace_all_copy(s, "C", "S") << '\n';
	std::cout << replace_head_copy(s, 3, "C++") << '\n';
	std::cout << replace_tail_copy(s, 5, "boost") << '\n';
	boost::iterator_range<std::string::iterator> r = find_first(s, "ec");
	std::cout << r << '\n';
	boost::iterator_range<std::string::iterator> x = find_last(s, "Ec");  
	std::cout << x << '\n';
	boost::iterator_range<std::string::iterator> k = find_tail(s, 5);  
	std::cout << k << '\n';
	boost::iterator_range<std::string::iterator> p = find_head(s, 4);    
	std::cout << p <<'\n';
}
