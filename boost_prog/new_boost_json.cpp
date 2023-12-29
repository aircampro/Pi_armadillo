#include <stdio.h>
#include <boost/json.hpp>
#include <iostream>
void test1(std::string &str)
{
    boost::json::object obj;
 obj["1"] = { {"v1","abc"}, 
                   {"v2","def"} ,
                   {"v3","ghi"},
                   {"v4",1234},
                   {"v5", 5678}
    };
    
    obj["2"] = "test";
    str = boost::json::serialize(obj);
    
}
void main(int argc, char* argv[])
{
    std::string str;
    test1(str);
 std::cout << str << std::endl; 
    parse the string
    boost::json::value val = boost::json::parse(str);
    if (val. is_object())
    {
        for (auto obj : val. as_object())
        {
            if (obj. key() == "1")
            {
                if (obj. value(). is_object() == false)
                {
                    break;
                }
                for (auto elm : obj. value(). as_object())
                {
                    if (elm. key() == "v1")
                    {
                        if (elm. value(). is_string() == false)
                        {
                            break;
                        }
 std::cout << elm. value() << std::endl; 
                    }
                    if (elm. key() == "v2")
                    {
                        if (elm. value(). is_string() == false)
                        {
                            break;
                        }
 std::cout << elm. value() << std::endl; 
                    }
                    if (elm. key() == "v3")
                    {
                        if (elm. value(). is_string() == false)
                        {
                            break;
                        }
 std::cout << elm. value() << std::endl; 
                    }
                    if (elm. key() == "v4")
                    {
                        if (elm. value(). is_number() == false)
                        {
                            break;
                        }
 std::cout << elm. value() << std::endl; 
                    }
                    if (elm. key() == "v5")
                    {
                        if (elm. value(). is_number() == false)
                        {
                            break;
                        }
 std::cout << elm. value() << std::endl; 
                    }
                }
            }
            if (obj. key() == "2")
            {
                if (obj. value(). is_string() == false)
                {
                    break;
                }
 std::cout << obj. value() << std::endl; 
            }
        }
    }
    getchar();
}
