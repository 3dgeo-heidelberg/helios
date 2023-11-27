#pragma once

#include <maths/EnergyMaths.h>

namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Energy models test
 */
class EnergyModelsTest : public BaseTest{
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
     * @brief Energy models test constructor
     */
    EnergyModelsTest() : BaseTest("Energy models test") {}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

    // ***  SUB-TESTS  *** //
    // ******************* //
    /**
     * @brief Test emitted received power.
     * @return True if passed, false otherwise.
     */
    bool testEmittedReceivedPower();
    /**
     * @brief Test emitted subray-wise power.
     * @return True if passed, false otherwise.
     */
    bool testEmittedSubrayWisePower();
};

// ***  R U N  *** //
// *************** //
bool EnergyModelsTest::run(){
    // Run tests
    if(!testEmittedReceivedPower()) return false;
    if(!testEmittedSubrayWisePower()) return false;
    return true;
}

// ***  SUB-TESTS  *** //
// ******************* //
bool EnergyModelsTest::testEmittedReceivedPower(){
    // Values for the tests
    std::vector<double> I0({450, 575, 250, 75, 370, 40, 30, 900});
    std::vector<double> lambda(
        {0.3, 0.6, 0.0001, 0.001, 0.005, 0.01, 0.05, 0.1}
    );
    std::vector<double> R({7.5, 6.3, 1.0, 5.0, 10.0, 15.0, 20.0, 30.0});
    std::vector<double> R0({8.0, 9.0, 0.1, 0.5, 1.0, 5.0, 10.0, 15.0});
    std::vector<double> r({0.33, 0.67, 0.09, 13, 0.55, 0.26, 0.19, 7.4});
    std::vector<double> w0({0.9, 0.6, 0.05, 0.1, 0.3, 1.0, 1.5, 1.3});
    std::vector<double> Dr2({0.9, 0.8, 0.1, 0.2, 0.7, 0.3, 0.6, 0.5});
    std::vector<double> Bt2({0.7, 0.6, 0.5, 0.4, 0.05, 0.1, 0.2, 0.3});
    std::vector<double> etaSys({1.1, 0.3, 0.6, 0.9, 1.2, 1.0, 0.5, 0.1});
    std::vector<double> ae({0.0, 0.05, 0.1, 0.15, 0.23, 0.17, 0.09, 0.009});
    std::vector<double> sigma({3.0, 0.5, 0.4, 0.3, 0.6, 0.7, 1.5, 2.0});

    // Compute tests
    size_t const nTests = I0.size();
    for(size_t i = 0 ; i < nTests ; ++i){
        // Test emitted power for i-th case
        double const PeNew = EnergyMaths::calcEmittedPower(
            I0[i], lambda[i], R[i], R0[i], r[i], w0[i]
        );
        double const PeOld = EnergyMaths::calcEmittedPowerLegacy(
            I0[i], lambda[i], R[i], R0[i], r[i], w0[i]
        );
        if(std::fabs(PeNew-PeOld) > eps) return false;
        // Test received power for i-th case
        double const PrNew = EnergyMaths::calcReceivedPower(
            I0[i], lambda[i], R[i], R0[i], r[i], w0[i],
            Dr2[i], Bt2[i], etaSys[i], ae[i], sigma[i]
        );
        double const PrOld = EnergyMaths::calcReceivedPowerLegacy(
            PeOld, Dr2[i], R[i], Bt2[i], etaSys[i],
            EnergyMaths::calcAtmosphericFactor(R[i], ae[i]),
            sigma[i]
        );
        if(std::fabs(PrNew-PrOld) > eps) return false;

    }
    return true;
}

bool EnergyModelsTest::testEmittedSubrayWisePower() {
    // Values for the tests
    double const I0 = 4.0;
    double const R = 1.0;
    double const beamDiv_mrad = 0.3;
    int const BSQ = 5;
    vector<int> circleIter({
        0, 1, 2, 3, 4
    });
    vector<int> nsr({
        1, 6, 12, 18, 25
    });
    vector<double> expectedPe({
        0.3075346144534569, 0.1313115395419632, 0.07979892703790642,
        0.04638110122372834, 0.022832322754653023,
    });

    // Compute emitted power and validate it
    for(int i = 0 ; i < nsr.size() ; ++i){
        double const Pe = EnergyMaths::calcSubrayWiseEmittedPower(
            I0,
            R,
            beamDiv_mrad,
            0.0,
            BSQ,
            circleIter[i],
            nsr[i]
        );
        double const absDiff = std::fabs(Pe-expectedPe[i]);
        if(absDiff > eps) return false;
    }
    // Return true if all emitted powers were as expected
    return true;
}


}