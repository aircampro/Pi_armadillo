#include <boost/exception/all.hpp>
#include <iostream>
#include <string>
#include <exception>
using namespace std;
 
class my_ex : public boost::exception, public std::exception {
public:
        string m_msg;
        string m_class = "my_ex";
 
        ~my_ex () throw() {}
 
        virtual const char* what() const throw() { return m_msg.c_str(); };
        my_ex(const string msg) : std::exception(), m_msg(msg) {}
        my_ex() : std::exception(), m_msg("my_ex") {}
};
 
int main(void)
{
        std::string msg = "Exception:: Im throwing ";
        try {
//                throw my_ex(msg);
                  BOOST_THROW_EXCEPTION( my_ex(msg) );
        } catch (my_ex &ex) {
                cerr << "my exception catch" << endl;
                cerr << ex.what () << " " << ex.m_class << " " << diagnostic_information(ex) << endl;
        } catch (exception &ex) {
                cerr << "std exception catch" << endl;
                cerr << ex.what () << endl;
        } catch (...) {
                cerr << "unknown" << endl;
        }
        cerr << "\033[35m at the bottom \033[0m" << endl;
 
        exit (0);
}
