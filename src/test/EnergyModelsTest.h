#pragma once

#include <maths/EnergyMaths.h>
#include <scanner/detector/FullWaveformPulseRunnable.h>

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
    /**
     * @brief Test the energy model considering the simulated elliptical
     *  footprint.
     * @return True if passed, false otherwise.
     */
    bool testEllipticalFootprintEnergy();

};

// ***  R U N  *** //
// *************** //
bool EnergyModelsTest::run(){
    // Run tests
    if(!testEmittedReceivedPower()) return false;
    if(!testEmittedSubrayWisePower()) return false;
    if(!testEllipticalFootprintEnergy()) return false;
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
    double const beamDiv = 0.3;
    double const w = beamDiv * R;
    int const BSQ = 5;
    vector<int> nsr({
        1, 6, 12, 18, 25
    });
    vector<double> expectedPe({
        0.02461531, 0.03192447, 0.02965419, 0.02622347, 0.0211937
    });

    // Compute emitted power and validate it
    int const n = nsr.size();
    for(int i = 0 ; i < n ; ++i){
        double const angle = beamDiv/2.0 * (i / (BSQ-0.5));
        double const prevAngle = (i == 0) ? 0.0 :
            beamDiv / 2.0 * ((i-1.0) / (BSQ-0.5));
        double const radius = angle + beamDiv/2.0 * (0.5 / (BSQ-0.5));
        double const prevRadius = (i == 0) ? 0.0 :
            prevAngle + beamDiv/2.0 * (0.5 / (BSQ-0.5));
        double const Pe = EnergyMaths::calcSubrayWiseEmittedPower(
            I0,
            w,
            radius,
            prevRadius,
            nsr[i]
        );
        double const absDiff = std::fabs(Pe-expectedPe[i]);
        if(absDiff > eps) return false;
    }
    // Return true if all emitted powers were as expected
    return true;
}

bool EnergyModelsTest::testEllipticalFootprintEnergy(){
#if DATA_ANALYTICS < 1
    // Prepare fake scanner
    std::shared_ptr<Scanner> scanner = std::make_shared<SingleScanner>(
        0.0003, // beamDiv_rad
        glm::dvec3(0, 0, 0), // beamOrigin,
        Rotation(), // beamOrientation
        std::list<int>({100000, 300000}),  // pulse freqs.
        5, // pulseLength_ns
        "scanDev0", // id
        4.0, // averagePower_w
        1.0, // beamQuality
        0.98999999999999, // efficiency
        0.15, // receiverDiameter_m
        9.07603791e-6, // atmosphericExtinction
        1.064e-06, // wavelength_m
        nullptr, // rangeErrExpr
        false, // Write waveform
        false, // Write pulse
        false, // Calc echowidth
        false, // Fullwave noise
        false // Platform noise disabled
    );
    std::shared_ptr<AbstractDetector> detector = std::make_shared<
        FullWaveformPulseDetector
    >(
        scanner,
        0.005, // accuracy_m
        0.01 // rangeMin_m
    );
    // Configure scanner to match expected values
    scanner->prepareSimulation();
    scanner->setDetector(detector);
    scanner->setNumRays(37);
    scanner->setWavelength(1064e-06);
    scanner->setBeamWaistRadius(0.0011289390629985112);
    scanner->setAtmosphericExtinction(9.07603791e-6);

    Material material;
    material.isGround = false;
    material.useVertexColors = false;
    material.map_Kd = "";
    material.reflectance = 0.5;
    material.specularity = 0;
    material.specularExponent = 96.078430999999;
    material.classification = 0;
    material.spectra = "shingle_red";
    float ka[4] = {1, 1, 1, 0};
    std::copy(std::begin(ka), std::end(ka), std::begin(material.ka));
    float kd[4] = {0.639999986, 0.639999986, 0.639999986, 0};
    std::copy(std::begin(kd), std::end(kd), std::begin(material.kd));
    float ks[4] = {0, 0, 0, 0};
    std::copy(std::begin(ks), std::end(ks), std::begin(material.ks));

    std::vector<double> raytracingRanges({
        0.1,        0.5,        1.0,        1.3,        1.5,
        2.1,        3.7,        5.0,        10.0,       22.3,
        28.8,       30.9,       51.7,       123.2,      900.318,
        1500.1,     1550,       1999.0,     1999.99,    2900
    });
    std::vector<double> incidenceAngles({  // in radians
        0.0,        0.01,       0.036,      0.1,        0.12,
        0.0,        0.1,        0.2,        0.3,        0.4,
        0.5,        0.4,        0.2,        0.1,        0.0,
        1.5,        1.3,        1.2,        1.0,        0.8
    });
    std::vector<double> divergenceAngles({  // in radians
        0.0003,     0.00003,    0.000003,   0.0000003,  0.003,
        0.0001,     0.00001,    0.000001,   0.0000001,  0.001,
        0.0002,     0.00002,    0.000002,   0.0000002,  0.002,
        0.015,      0.0015,     0.00015,    0.000015,   0.15,
    });
    std::vector<double> expectedIntensities({
        94565825.352190837, 3782423.834751925,
        945031.965353, 556754.596874, 417176.862884,
        214427.570343, 68727.120275, 37069.098079, 9032.612891, 1750.769938,
        1000.026362, 911.724653, 346.419866, 61.854047, 1.147646,
        0.0287837, 0.102365, 0.0826968, 0.123183, 0.0452485
    });
    for(size_t i = 0 ; i < raytracingRanges.size(); ++i){
        double const raytracingRange = raytracingRanges[i];
        double const incidenceAngle = incidenceAngles[i];
        double const divergenceAngle = divergenceAngles[i];
        double const radius = std::sin(divergenceAngle) * raytracingRange;
        double const intensity = scanner->calcIntensity(
            incidenceAngle, raytracingRange, material,
            radius, 0, 0
        );
        double const expectedIntensity = expectedIntensities[i];
        double const absDiff = std::fabs(expectedIntensity-intensity);
        if(absDiff > eps) return false;
    }
#endif
    // Return true if all intensities were as expected
    return true;
}


}