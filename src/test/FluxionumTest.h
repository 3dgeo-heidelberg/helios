#pragma once

#include <fluxionum/UnivariateNewtonRaphsonMinimizer.h>

#include <functional>
#include <cmath>

namespace HeliosTests{

using namespace fluxionum;
using std::function;

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief Fluxionum test
 */
class FluxionumTest : public BaseTest{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Decimal precision for validation purposes
     */
    double const eps = 0.00001;

    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief Fluxionum test constructor
     */
    FluxionumTest() : BaseTest("Fluxionum test") {}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

    // ***  SUB-TESTS  *** //
    // ******************* //
    /**
     * @brief Test univariate Newton-Raphson minimization
     * @return True if passed, false otherwise
     */
    bool testUnivariateNewtonRaphsonMinimization();
};

// ***  R U N  *** //
// *************** //
bool FluxionumTest::run(){
    // Run tests
    if(!testUnivariateNewtonRaphsonMinimization()) return false;
    return true;
}

// ***  SUB-TESTS  *** //
// ******************* //
bool FluxionumTest::testUnivariateNewtonRaphsonMinimization(){
    double expected = -3.0;
    UnivariateNewtonRaphsonMinimizer<double, double> unrm(
        [] (double x) -> double {return std::pow(x, 2)/2.0 + 3.0*x;},
        [] (double x) -> double {return x+3.0;},
        [] (double x) -> double {return 1;}
    );
    double x = unrm.argmin(0.0);
    return std::fabs(x-expected) <= eps;
}

}