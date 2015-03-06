/*
 *   test_class.hpp
 *
 *   Author: Chien-Ming Lin
 *
 */
#ifndef _TEST_CLASS_H_
#define _TEST_CLASS_H_

#include <string.h>
#include "ros/ros.h""


namespace test
{
class test_class
{
private:
    static test_class* m_UniqueInstance;

    const double mAngle1Sin;
    const double mAngle2Sin;
    const double mAngle3Sin;

    const double mAngle1Cos;
    const double mAngle2Cos;
    const double mAngle3Cos;

public:
    static test_class* GetInstance() {
        return m_UniqueInstance;
    }

    test_class();
    ~test_class();

    void Initialize();
    void Process();
};
}

#endif
