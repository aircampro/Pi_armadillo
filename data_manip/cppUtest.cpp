// ref ：https://cpputest.github.io/manual.html#test_macros

#include "CppUTest/CommandLineTestRunner.h"
#include <climits>

int inc(int x){
    return x+1;
}

TEST_GROUP(TestFuncInc){
};


TEST(TestFuncInc, CheckReturnValue){
      INT_EQUAL(11, inc(10))                              // first 3 pass then fail
      INT_EQUAL(6000, inc(5999))
      INT_EQUAL(INT_MAX, inc(INT_MAX-1))
      INT_EQUAL(INT_MAX+1, inc(INT_MAX))
      INT_EQUAL(6000, inc(800))
}


// main() 
int main(int argc, char **argv){
      return CommandLineTestRunner::RunAllTests(argc, argv);
}

