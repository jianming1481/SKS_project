#include "test_class.hpp"
#include "iostream"
#include "ros/ros.h"

using namespace std;
using namespace test;


void test_class::Process()
{
    std::cout << "hello_hpp" << std::endl;
}

int main(int argc,char** argv)
{
    //ros::init(argc,argv,"test_class"); //You can not do this!
    test::test_class* fuck;
    std::cout << "hello_cpp" << std::endl;
    fuck->Process();

    return 0;
}
