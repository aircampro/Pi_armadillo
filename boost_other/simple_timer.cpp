#include <iostream>
#include <string>
 
#define BOOST  
 
#if defined(BOOST)
#include <boost/optional.hpp> 
#include <cstdlib>
#include <boost/timer/timer.hpp>
#elif defined(SPROUT)
#include <sprout/optional.hpp> 
#endif
 
#if defined(BOOST)
boost::optional<int> f(int n)
{
	    return { n };
}
#elif defined(SPROUT)
sprout::optional<int> f(int n)
{
	    return { n };
}
#endif
 
int main()
{
#if defined(BOOST)
	    const auto n1 = f(23);
	     
	        if (n1) std::cout << "note its a pointer " << *n1 << std::endl;
		 
		    const long n=10000000;
		        {
				        boost::timer::cpu_timer timer;
					 
					        for (long i = 0; i < n; ++i) {

							        }
						 
						        std::string result = timer.format();
							        std::cout << result << std::endl;
								    }
			    {
				            boost::timer::cpu_timer timer;
					     
					            for (long i = 0; i < n; ++i) {
							            }
						            
						            std::string result = timer.format();
							            std::cout << result << std::endl;
								        }
			     
#elif defined(SPROUT)
			        const auto n = f(23);
				    for (int i : n) std::cout << "the value using sprout " << i << std::endl;
#endif
				     
}
