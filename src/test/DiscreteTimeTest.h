#pragma once

#include <BaseTest.h>
#include <sim/tools/DiscreteTime.h>

namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Test discrete time handling
 */
class DiscreteTimeTest : public BaseTest{
public:
    // ***  CONSTRUCTOR  *** //
    // ********************* //
    DiscreteTimeTest() : BaseTest("Discrete time test") {}
    virtual ~DiscreteTimeTest(){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;
};

// ***  R U N  *** //
// *************** //
bool DiscreteTimeTest::run(){
    // Do some tests
    DiscreteTime dt1(500, 1.0);
    if(dt1.toDiscrete(0.5)!=250) return false;
    if(dt1.toDiscrete(1.5)!=750) return false;
    if(dt1.toCyclicDiscrete(0.5)!=250) return false;
    if(dt1.toCyclicDiscrete(1.5)!=250) return false;
    if(dt1.toCyclicDiscrete(3.5)!=250) return false;
    if(dt1.toPeriodicDiscrete(0.5)!=250) return false;
    if(dt1.toPeriodicDiscrete(1.5)!=250) return false;
    if(dt1.toPeriodicDiscrete(3.5)!=250) return false;
    if(dt1.toContinuous(250)!=0.5) return false;
    if(dt1.toContinuous(750)!=1.5) return false;
    if(dt1.toCyclicContinuous(250)!=0.5) return false;
    if(dt1.toCyclicContinuous(750)!=0.5) return false;
    if(dt1.toCyclicContinuous(1750)!=0.5) return false;
    if(dt1.toPeriodicContinuous(250)!=0.5) return false;
    if(dt1.toPeriodicContinuous(750)!=0.5) return false;
    if(dt1.toPeriodicContinuous(1750)!=0.5) return false;

    // Do some more tests
    DiscreteTime dt2(500, 2.0);
    if(dt2.toDiscrete(0.5)!=250) return false;
    if(dt2.toDiscrete(1.5)!=750) return false;
    if(dt2.toCyclicDiscrete(0.5)!=250) return false;
    if(dt2.toCyclicDiscrete(1.5)!=250) return false;
    if(dt2.toCyclicDiscrete(3.5)!=250) return false;
    if(dt2.toPeriodicDiscrete(0.0)!=0) return false;
    if(dt2.toPeriodicDiscrete(0.2)!=100) return false;
    if(dt2.toPeriodicDiscrete(0.5)!=250) return false;
    if(dt2.toPeriodicDiscrete(1.5)!=750) return false;
    if(dt2.toPeriodicDiscrete(2.0)!=0) return false;
    if(dt2.toPeriodicDiscrete(2.2)!=100) return false;
    if(dt2.toPeriodicDiscrete(2.5)!=250) return false;
    if(dt2.toPeriodicDiscrete(3.5)!=750) return false;
    if(dt2.toContinuous(250)!=0.5) return false;
    if(dt2.toContinuous(750)!=1.5) return false;
    if(dt2.toCyclicContinuous(250)!=0.5) return false;
    if(dt2.toCyclicContinuous(750)!=0.5) return false;
    if(dt2.toCyclicContinuous(1750)!=0.5) return false;
    if(dt2.toPeriodicContinuous(0)!=0.0) return false;
    if(dt2.toPeriodicContinuous(100)!=0.2) return false;
    if(dt2.toPeriodicContinuous(250)!=0.5) return false;
    if(dt2.toPeriodicContinuous(500)!=1.0) return false;
    if(dt2.toPeriodicContinuous(750)!=1.5) return false;
    if(dt2.toPeriodicContinuous(1000)!=0.0) return false;
    if(dt2.toPeriodicContinuous(1250)!=0.5) return false;
    if(dt2.toPeriodicContinuous(1500)!=1.0) return false;
    if(dt2.toPeriodicContinuous(1750)!=1.5) return false;

    // Do even more tests
    DiscreteTime dt3(500, 0.5);
    if(dt3.toDiscrete(0.5)!=250) return false;
    if(dt3.toDiscrete(1.5)!=750) return false;
    if(dt3.toCyclicDiscrete(0.5)!=250) return false;
    if(dt3.toCyclicDiscrete(1.5)!=250) return false;
    if(dt3.toCyclicDiscrete(3.5)!=250) return false;
    if(dt3.toPeriodicDiscrete(0.0)!=0) return false;
    if(dt3.toPeriodicDiscrete(0.2)!=100) return false;
    if(dt3.toPeriodicDiscrete(0.5)!=0) return false;
    if(dt3.toPeriodicDiscrete(1.5)!=0) return false;
    if(dt3.toPeriodicDiscrete(2.0)!=0) return false;
    if(dt3.toPeriodicDiscrete(2.2)!=100) return false;
    if(dt3.toPeriodicDiscrete(2.5)!=0) return false;
    if(dt3.toPeriodicDiscrete(3.5)!=0) return false;
    if(dt3.toContinuous(250)!=0.5) return false;
    if(dt3.toContinuous(750)!=1.5) return false;
    if(dt3.toCyclicContinuous(250)!=0.5) return false;
    if(dt3.toCyclicContinuous(750)!=0.5) return false;
    if(dt3.toCyclicContinuous(1750)!=0.5) return false;
    if(dt3.toPeriodicContinuous(0)!=0.0) return false;
    if(dt3.toPeriodicContinuous(100)!=0.2) return false;
    if(dt3.toPeriodicContinuous(250)!=0.0) return false;
    if(dt3.toPeriodicContinuous(500)!=0.0) return false;
    if(dt3.toPeriodicContinuous(750)!=0.0) return false;
    if(dt3.toPeriodicContinuous(1000)!=0.0) return false;
    if(dt3.toPeriodicContinuous(1250)!=0.0) return false;
    if(dt3.toPeriodicContinuous(1350)!=0.2) return false;
    if(dt3.toPeriodicContinuous(1500)!=0.0) return false;
    if(dt3.toPeriodicContinuous(1750)!=0.0) return false;

    // Yet more tests
    if(dt1.toDiscrete(0.00001) != 0) return false;
    if(dt2.toCyclicDiscrete(0.00001) != 0) return false;
    if(dt2.toPeriodicDiscrete(0.00001) != 0) return false;
    if(dt1.toContinuous(0) != 0.0) return false;
    if(dt1.toContinuous(1) <= 0.0) return false;
    if(dt3.toCyclicContinuous(0) != 0.0) return false;
    if(dt3.toPeriodicContinuous(1) <= 0.0) return false;
    if(dt2.toDiscrete(0.99999999) != 499) return false;
    if(std::fabs(dt3.toContinuous(499)-0.999999) < 0.000001) return false;
    if(dt2.toPeriodicDiscrete(1.99999999) != 999) return false;
    if(dt3.toPeriodicDiscrete(0.49999999) != 249) return false;
    if(std::fabs(dt2.toPeriodicContinuous(999)-1.9999) < 0.0001) return false;
    if(std::fabs(dt3.toPeriodicContinuous(249)-0.4999) < 0.0001) return false;

    // Another bunch of tests
    DiscreteTime dt4(1000000, 1.0);
    if(dt4.toDiscrete(0.00000000001) != 0) return false;
    if(dt4.toDiscrete(0.000001) != 1) return false;
    if(dt4.toDiscrete(0.999999999999) != 999999) return false;
    if(dt4.toDiscrete(3.00000000001) != 3000000) return false;
    if(dt4.toDiscrete(2.000001) != 2000001) return false;
    if(dt4.toDiscrete(4.999999999999) != 4999999) return false;
    if(dt4.toCyclicDiscrete(3.00000000001) != 0) return false;
    if(dt4.toCyclicDiscrete(2.000001) != 1) return false;
    if(dt4.toCyclicDiscrete(4.999999999999) != 999999) return false;
    if(dt4.toPeriodicDiscrete(3.00000000001) != 0) return false;
    if(dt4.toPeriodicDiscrete(2.000001) != 1) return false;
    if(dt4.toPeriodicDiscrete(4.999999999999) != 999999) return false;
    if(dt4.toContinuous(0) != 0.0) return false;
    if(dt4.toContinuous(1) != 0.000001) return false;
    if(std::fabs(dt4.toContinuous(999999)-0.999999) > 0.0000001) return false;
    if(std::fabs(dt4.toContinuous(3000000)) != 3.0) return false;
    if(std::fabs(dt4.toContinuous(2000001)-2.000001) > 0.0000001) return false;
    if(std::fabs(dt4.toContinuous(4999999)-4.999999) > 0.0000001) return false;
    if(std::fabs(dt4.toCyclicContinuous(3000000)) != 0.0) return false;
    if(std::fabs(dt4.toCyclicContinuous(2000001)-0.000001) > 0.0000001)
        return false;
    if(std::fabs(dt4.toCyclicContinuous(4999999)-0.999999) > 0.0000001)
        return false;
    if(std::fabs(dt4.toPeriodicContinuous(3000000)) != 0.0) return false;
    if(std::fabs(dt4.toPeriodicContinuous(2000001)-0.000001) > 0.0000001)
        return false;
    if(std::fabs(dt4.toPeriodicContinuous(4999999)-0.999999) > 0.0000001)
        return false;
    if(dt4.toDiscrete(dt4.toContinuous(0))!=0) return false;
    if(dt4.toDiscrete(dt4.toContinuous(500000))!=500000) return false;
    if(dt4.toDiscrete(dt4.toContinuous(999999))!=999999) return false;
    if(dt4.toDiscrete(dt4.toContinuous(1000000))!=1000000) return false;
    if(dt4.toCyclicDiscrete(dt4.toContinuous(1000000))!=0) return false;
    if(dt4.toPeriodicDiscrete(dt4.toContinuous(1000000))!=0) return false;
    if(dt4.toContinuous(dt4.toDiscrete(0.0))!=0.0) return false;
    if(dt4.toContinuous(dt4.toDiscrete(0.5))!=0.5) return false;
    if(dt4.toContinuous(dt4.toDiscrete(1.0))!=1.0) return false;
    if(dt4.toCyclicContinuous(dt4.toDiscrete(1.0))!=0.0) return false;
    if(dt4.toPeriodicContinuous(dt4.toDiscrete(1.0))!=0.0) return false;

    // All tests were successful
    return true;
}

}