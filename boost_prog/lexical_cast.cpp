#include <iostream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

std::string boost_string_from_int( int val ) {
    return boost::lexical_cast<std::string>(val);
}


std::string boost_string_from_double( double val ) {
    return boost::lexical_cast<std::string>(val);
}


boost::optional<std::int32_t> boost_string_to_int( std::string s ) {
    try {
       return boost::lexical_cast<std::int32_t>(s);
    } catch(boost::bad_lexical_cast &e) {
       std::cout << s << "(error): " << e.what() << "\n";
       return boost::none;
    }
}

// use an any type template
//
template<typename Target, typename Source>
inline boost::optional<Target> lexical_cast_opt( Source const& src )
{
  try
  {
    return boost::lexical_cast<Target>( src );
  }
  catch( boost::bad_lexical_cast &e)
  {
    std::cout << src << "(error): " << e.what() << "\n";
    return boost::none;
  }
}

int main(void) {

   std::string ss = "1235";
   //std::int32_t yy = boost_string_to_int(ss);
   boost::optional<int> yy = boost_string_to_int(ss);
   int xx = 1234;
   std::string sss = boost_string_from_int(xx);
   double dub = 0.0345765;
   std::string ss2 = boost_string_from_double(dub);
   std::string s2s = "1235.3";
   boost::optional<int> yy1 = boost_string_to_int(s2s);
   auto anytyp = lexical_cast_opt<double>(s2s);
   std::string s3s = "   1235.3";
   boost::optional<int> yy2 = boost_string_to_int(s3s);
   auto anytyp2 = lexical_cast_opt<double>(s3s);
   boost::algorithm::trim( s3s );
   boost::optional<int> yy3 = boost_string_to_int(s3s);
   auto anytyp3 = lexical_cast_opt<double>(s3s);
   std::cout << sss << " " << ss2 << "   " << yy  << " " << yy1 << " " << anytyp << "\n";
   std::cout <<  yy2  << " " << yy3 << " " << anytyp2 << " " << anytyp3 << "\n";
   return 0;
}
