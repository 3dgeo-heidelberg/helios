#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <maths/EnergyMaths.h>
#include <scanner/detector/FullWaveformPulseRunnable.h>

TEST_CASE( "Energy Models Test ") {
    double const eps = 0.00001;

    SECTION("Test Emitted Received Power") {
        // Generate values for the tests
        // Format: I0, lambda, R, R0, r, w0, Dr2, Bt2, etaSys, ae, sigma
        auto [I0, lambda, R, R0, r, w0, Dr2, Bt2, etaSys, ae, sigma] = GENERATE(
            std::make_tuple(450., 0.3, 7.5, 5.0, 0.33, 0.9, 0.9, 0.7, 1.1, 0.0, 3.0),
            std::make_tuple(575., 0.6, 6.3, 9.0, 0.67, 0.6, 0.8, 0.6, 0.3, 0.05, 0.5),
            std::make_tuple(250., 0.0001, 1.0, 0.1, 0.09, 0.05, 0.1, 0.5, 0.6, 0.1, 0.4),
            std::make_tuple(75., 0.001, 5.0, 0.5, 13., 0.1, 0.2, 0.4, 0.9, 0.15, 0.3),
            std::make_tuple(370., 0.005, 10.0, 1.0, 0.55, 0.3, 0.7, 0.05, 1.2, 0.23, 0.6),
            std::make_tuple(40., 0.01, 15.0, 5.0, 0.26, 1.0, 0.3, 0.1, 1.0, 0.17, 0.7),
            std::make_tuple(30., 0.05, 20.0, 10.0, 0.19, 1.5, 0.6, 0.2, 0.5, 0.09, 1.5),
            std::make_tuple(900., 0.1, 30.0, 15.0, 7.4, 1.3, 0.5, 0.3, 0.1, 0.009, 2.0)
        );

        // Compute tests
        // Test emitted power for i-th case
        double const PeNew = EnergyMaths::calcEmittedPower(
            I0, lambda, R, R0, r, w0
        );
        double const PeOld = EnergyMaths::calcEmittedPowerLegacy(
            I0, lambda, R, R0, r, w0
        );
        REQUIRE(std::fabs(PeNew-PeOld) <= eps);
        // Test received power for i-th case
        double const PrNew = EnergyMaths::calcReceivedPower(
            I0, lambda, R, R0, r, w0,
            Dr2, Bt2, etaSys, ae, sigma
        );
        double const PrOld = EnergyMaths::calcReceivedPowerLegacy(
            PeOld, Dr2, R, Bt2, etaSys,
            EnergyMaths::calcAtmosphericFactor(R, ae),
            sigma
        );
        REQUIRE(std::fabs(PrNew-PrOld) <= eps);
    }

    SECTION("Test Emitted Subray Wise Power") {
        // Values for the tests
        double const I0 = 499504.174245084;
        double const beamDiv = 0.0003;
        double const w = 0.01288227007085636;
        double const w0 = 0.0022578781259970223;
        int const BSQ = 8;

        auto [i, nsr, expectedPe] = GENERATE(
            std::make_tuple(0, 1, 4.820642506508932e-06),
            std::make_tuple(1, 6, 6.427488484487635e-06),
            std::make_tuple(2, 12, 6.427395531249935e-06),
            std::make_tuple(3, 18, 6.4272406122957276e-06),
            std::make_tuple(4, 25, 6.16994278269445e-06),
            std::make_tuple(5, 31, 6.219430545125497e-06),
            std::make_tuple(6, 37, 6.252717516617577e-06),
            std::make_tuple(7, 43, 6.276559501937178e-06)
        );

        // Compute emitted power and validate it
        double const angle = beamDiv/2.0 * (i / (BSQ-0.5));
        double const prevAngle = (i == 0) ? 0.0 :
            beamDiv / 2.0 * ((i-1.0) / (BSQ-0.5));
        double const radius = angle + beamDiv/2.0 * (0.5 / (BSQ-0.5));
        double const prevRadius = (i == 0) ? 0.0 :
            prevAngle + beamDiv/2.0 * (0.5 / (BSQ-0.5));
        double const Pe = EnergyMaths::calcSubrayWiseEmittedPower(
            I0,
            w0,
            w,
            radius,
            prevRadius,
            nsr
        );
        double const absDiff = std::fabs(Pe-expectedPe);
        REQUIRE(absDiff <= eps);
    }

    SECTION("Test Elliptical Footprint Energy") {
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
            false, // Write waveform
            false, // Write pulse
            false, // Calc echowidth
            false, // Fullwave noise
            false, // Platform noise disabled
            nullptr // rangeErrExpr
        );
        std::shared_ptr<AbstractDetector> detector = std::make_shared<
            FullWaveformPulseDetector
        >(
            scanner,
            0.005, // accuracy_m
            0.01 // rangeMin_m
        );
        // Configure scanner to match expected values
        scanner->setDetector(detector);
        scanner->setNumRays(37);
        scanner->setWavelength(1064e-06);
        scanner->setBeamWaistRadius(0.0011289390629985112);
        scanner->setAtmosphericExtinction(9.07603791e-6);
        scanner->prepareSimulation(false);
        // Get reference to scanning device
        ScanningDevice &scanDev = scanner->getScanningDevice(0);

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

        auto [raytracingRange, incidenceAngle, divergenceAngle, expectedIntensity] = GENERATE(
            std::make_tuple(0.1, 0.0, 0.0003, 0.061007356437843892),
            std::make_tuple(0.5, 0.01, 0.00003, 8.231786690673595e-05),
            std::make_tuple(1.0, 0.036, 0.000003, 5.0388698965697264e-06),
            std::make_tuple(1.3, 0.1, 0.0000003, 1.7473066265510948e-06),
            std::make_tuple(1.5, 0.12, 0.003, 9.8214055513872063e-07),
            std::make_tuple(2.1, 0.0, 0.0001, 2.5632187072077593e-07),
            std::make_tuple(3.7, 0.1, 0.00001, 2.6255666684330216e-08),
            std::make_tuple(5.0, 0.2, 0.000001, 7.7659421201791914e-09),
            std::make_tuple(10.0, 0.3, 0.0000001, 4.8235713209654749e-10),
            std::make_tuple(22.3, 0.4, 0.001, 2.1576050021742726e-11),
            std::make_tuple(28.8, 0.5, 0.0002, 8.2158742656362774e-12),
            std::make_tuple(30.9, 0.4, 0.00002, 3.7452055307437231e-12),
            std::make_tuple(51.7, 0.2, 0.000002, 0.),
            std::make_tuple(123.2, 0.1, 0.0000002, 0.),
            std::make_tuple(900.318, 0.0, 0.002, 0.),
            std::make_tuple(1500.1, 1.5, 0.015, 0.),
            std::make_tuple(1550., 1.3, 0.0015, 0.),
            std::make_tuple(1999.0, 1.2, 0.00015, 0.),
            std::make_tuple(1999.99, 1.0, 0.000015, 0.),
            std::make_tuple(2900., 0.8, 0.15, 0.)
        );
        for(size_t j = 0 ; j < scanDev.cached_subrayDivergenceAngle_rad.size() ; ++j){
            scanDev.cached_subrayDivergenceAngle_rad[j] = divergenceAngle;
        }
        double const intensity = scanner->calcIntensity(
            incidenceAngle, raytracingRange, material, 0, 0
        );
        REQUIRE(!isnan(intensity));
        double const absDiff = std::fabs(expectedIntensity-intensity)*1e09;
        REQUIRE(absDiff <= eps);
#endif
    }
}