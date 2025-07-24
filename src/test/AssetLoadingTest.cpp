#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

bool    logging::LOGGING_SHOW_TRACE,    logging::LOGGING_SHOW_DEBUG,
        logging::LOGGING_SHOW_INFO,     logging::LOGGING_SHOW_TIME,
        logging::LOGGING_SHOW_WARN,     logging::LOGGING_SHOW_ERR;

#include <assetloading/XmlUtils.h>
#include <assetloading/XmlSceneLoader.h>
#include <assetloading/XmlAssetsLoader.h>
#include <assetloading/XmlSurveyLoader.h>
#include <scanner/beamDeflector/OscillatingMirrorBeamDeflector.h>
#include <scanner/beamDeflector/ConicBeamDeflector.h>
#include <scanner/beamDeflector/PolygonMirrorBeamDeflector.h>
#include <scanner/beamDeflector/RisleyBeamDeflector.h>
#include <scanner/SingleScanner.h>
#include <scanner/MultiScanner.h>
#include <platform/Platform.h>
#include <platform/HelicopterPlatform.h>
#include <platform/LinearPathPlatform.h>
#include <platform/GroundVehiclePlatform.h>
#include <noise/NoiseSource.h>
#include <noise/NormalNoiseSource.h>
#include <noise/UniformNoiseSource.h>

TEST_CASE("Asset Loading Tests") {
    double const eps = 0.00001;

    SECTION("Test Scanner Loading") {
        std::string testScannersPath = "data/test/test_scanners.xml";
        std::vector<std::string> assetsPath = { "assets/" };
        XmlAssetsLoader loader(testScannersPath, assetsPath);

        // Leica ALS50
        std::shared_ptr<Scanner> scanner = std::static_pointer_cast<Scanner>(
            loader.getAssetById("scanner", "leica_als50", nullptr)
        );
        REQUIRE(std::dynamic_pointer_cast<SingleScanner>(scanner));
        REQUIRE_FALSE(std::dynamic_pointer_cast<MultiScanner>(scanner));
        REQUIRE(scanner->getScannerId() == "leica_als50");
        REQUIRE(scanner->getDetector()->cfg_device_accuracy_m == 0.05);
        REQUIRE(scanner->getBeamDivergence() == 0.00033);
        REQUIRE(scanner->name == "Leica ALS50");
        REQUIRE(std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(scanner->getBeamDeflector()));
        auto pfs = scanner->getSupportedPulseFreqs_Hz();
        REQUIRE(pfs.size() == 1);
        REQUIRE(pfs.front() == 83000);
        REQUIRE(scanner->getPulseFreq_Hz() == 83000);
        REQUIRE(scanner->getPulseLength_ns() == 10);
        REQUIRE(scanner->getDetector()->cfg_device_rangeMin_m == 200);
        REQUIRE(std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.6544985) <= eps);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz == 25);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz == 70);

        // Leica ALS50-II
        scanner = std::static_pointer_cast<Scanner>(
            loader.getAssetById("scanner", "leica_als50-ii", nullptr)
        );
        REQUIRE(std::dynamic_pointer_cast<SingleScanner>(scanner));
        REQUIRE_FALSE(std::dynamic_pointer_cast<MultiScanner>(scanner));
        REQUIRE(scanner->getScannerId() == "leica_als50-ii");
        REQUIRE(scanner->getDetector()->cfg_device_accuracy_m == 0.05);
        REQUIRE(scanner->getBeamDivergence() == 0.00022);
        REQUIRE(scanner->name == "Leica ALS50-II");
        REQUIRE(std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(scanner->getBeamDeflector()));
        pfs = scanner->getSupportedPulseFreqs_Hz();
        REQUIRE(pfs.size() == 3);
        std::list<int>::iterator pfsit = pfs.begin();
        REQUIRE(*pfsit++ == 20000);
        REQUIRE(*pfsit++ == 60000);
        REQUIRE(*pfsit++ == 150000);
        REQUIRE(scanner->getPulseFreq_Hz() == 20000);
        REQUIRE(scanner->getPulseLength_ns() == 10);
        REQUIRE(scanner->getDetector()->cfg_device_rangeMin_m == 200);
        REQUIRE(std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.6544985) <= eps);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz == 0);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz == 90);
        REQUIRE(scanner->getMaxNOR() == 4);

        // Optech 2033
        scanner = std::static_pointer_cast<Scanner>(
            loader.getAssetById("scanner", "optech_2033", nullptr)
        );
        REQUIRE(std::dynamic_pointer_cast<SingleScanner>(scanner));
        REQUIRE_FALSE(std::dynamic_pointer_cast<MultiScanner>(scanner));
        REQUIRE(scanner->getScannerId() == "optech_2033");
        REQUIRE(scanner->getDetector()->cfg_device_accuracy_m == 0.01);
        REQUIRE(scanner->getBeamDivergence() == 0.000424);
        REQUIRE(scanner->name == "Optech ALTM 2033");
        REQUIRE(std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(scanner->getBeamDeflector()));
        pfs = scanner->getSupportedPulseFreqs_Hz();
        REQUIRE(pfs.size() == 1);
        REQUIRE(pfs.front() == 33000);
        REQUIRE(scanner->getPulseLength_ns() == 8);
        REQUIRE(scanner->getDetector()->cfg_device_rangeMin_m == 1);
        REQUIRE(std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.3490659) <= eps);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz == 0);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz == 30);
        REQUIRE(std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(scanner->getBeamDeflector())->cfg_device_scanProduct == 590);
        REQUIRE(scanner->getWavelength() == 1047e-9);
        glm::dvec3 epdiff = scanner->getHeadRelativeEmitterPosition() - glm::dvec3(0, 0.085, 0.06);
        REQUIRE(std::fabs(epdiff[0]) <= eps);
        REQUIRE(std::fabs(epdiff[1]) <= eps);
        REQUIRE(std::fabs(epdiff[2]) <= eps);
        Rotation eatt = scanner->getHeadRelativeEmitterAttitude();
        REQUIRE(eatt.getQ0() == 1);
        REQUIRE(eatt.getQ1() == 0);
        REQUIRE(eatt.getQ2() == 0);
        REQUIRE(eatt.getQ3() == 0);
        glm::dvec3 eraxis = scanner->getScannerHead()->cfg_device_rotateAxis;
        REQUIRE(eraxis[0] == 0);
        REQUIRE(eraxis[1] == 0);
        REQUIRE(eraxis[2] == 1);

        // Riegl LMS-Q560
        scanner = std::static_pointer_cast<Scanner>(
            loader.getAssetById("scanner", "riegl_lms-q560")
        );
        REQUIRE(std::dynamic_pointer_cast<SingleScanner>(scanner));
        REQUIRE_FALSE(std::dynamic_pointer_cast<MultiScanner>(scanner));
        REQUIRE(scanner->getScannerId() == "riegl_lms-q560");
        REQUIRE(scanner->getDetector()->cfg_device_accuracy_m == 0.02);
        REQUIRE(scanner->getBeamDivergence() == 0.0005);
        REQUIRE(scanner->name == "RIEGL LMS-Q560");
        REQUIRE(std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(scanner->getBeamDeflector()));
        pfs = scanner->getSupportedPulseFreqs_Hz();
        REQUIRE(pfs.size() == 5);
        pfsit = pfs.begin();
        REQUIRE(*pfsit++ == 50000);
        REQUIRE(*pfsit++ == 100000);
        REQUIRE(*pfsit++ == 180000);
        REQUIRE(*pfsit++ == 200000);
        REQUIRE(*pfsit++ == 240000);
        REQUIRE(scanner->getPulseLength_ns() == 4);
        REQUIRE(std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.7853982) <= eps);
        REQUIRE(std::fabs(std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(scanner->getBeamDeflector())->getScanAngleEffectiveMax_rad() - 0.3926991) <= eps);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz == 10);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz == 160);

        // RIEGL VQ-880-G
        scanner = std::static_pointer_cast<Scanner>(
            loader.getAssetById("scanner", "riegl_vq-880g", nullptr)
        );
        REQUIRE(std::dynamic_pointer_cast<SingleScanner>(scanner));
        REQUIRE_FALSE(std::dynamic_pointer_cast<MultiScanner>(scanner));
        REQUIRE(scanner->getScannerId() == "riegl_vq-880g");
        REQUIRE(scanner->getDetector()->cfg_device_accuracy_m == 0.025);
        REQUIRE(scanner->getBeamDivergence() == 0.0003);
        REQUIRE(scanner->name == "RIEGL VQ-880-G");
        REQUIRE(std::dynamic_pointer_cast<ConicBeamDeflector>(scanner->getBeamDeflector()));
        pfs = scanner->getSupportedPulseFreqs_Hz();
        REQUIRE(pfs.size() == 4);
        pfsit = pfs.begin();
        REQUIRE(*pfsit++ == 150000);
        REQUIRE(*pfsit++ == 300000);
        REQUIRE(*pfsit++ == 600000);
        REQUIRE(*pfsit++ == 900000);
        REQUIRE(scanner->getPulseLength_ns() == 2);
        REQUIRE(std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.3490659) <= eps);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz == 28);
        REQUIRE(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz == 200);
        REQUIRE(std::fabs(scanner->getWavelength()-1064e-9) <= eps);
        epdiff = scanner->getHeadRelativeEmitterPosition() - glm::dvec3(0, 0.085, 0.06);
        REQUIRE(std::fabs(epdiff[0]) <= eps);
        REQUIRE(std::fabs(epdiff[1]) <= eps);
        REQUIRE(std::fabs(epdiff[2]) <= eps);
        eatt = scanner->getHeadRelativeEmitterAttitude();
        REQUIRE(eatt.getQ0() == 1);
        REQUIRE(eatt.getQ1() == 0);
        REQUIRE(eatt.getQ2() == 0);
        REQUIRE(eatt.getQ3() == 0);
        eraxis = scanner->getScannerHead()->cfg_device_rotateAxis;
        REQUIRE(eraxis[0] == 0);
        REQUIRE(eraxis[1] == 0);
        REQUIRE(eraxis[2] == 1);

        // Livox Mid-70
        scanner = std::static_pointer_cast<Scanner>(
            loader.getAssetById("scanner", "livox_mid-70", nullptr)
        );
        REQUIRE(std::dynamic_pointer_cast<SingleScanner>(scanner));
        REQUIRE_FALSE(std::dynamic_pointer_cast<MultiScanner>(scanner));
        REQUIRE(scanner->getScannerId() == "livox_mid-70");
        REQUIRE(scanner->getDetector()->cfg_device_accuracy_m == 0.02);
        REQUIRE(scanner->getBeamDivergence() == 0.004887);
        REQUIRE(scanner->name == "Livox Mid-70");
        REQUIRE(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector()));
        pfs = scanner->getSupportedPulseFreqs_Hz();
        REQUIRE(pfs.size() == 1);
        REQUIRE(pfs.front() == 100000);
        REQUIRE(scanner->getPulseLength_ns() == 4);
        REQUIRE(scanner->getDetector()->cfg_device_rangeMin_m == 2);
        REQUIRE(std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.6108652) <= eps);
        REQUIRE(std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector())->rotorSpeed_rad_1 - 1160.876155) <= eps); // Hz / (2 pi)
        REQUIRE(std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector())->rotorSpeed_rad_2 + 742.298655) <= eps); // Hz / (2 pi)
        REQUIRE(std::fabs(scanner->getWavelength()-905e-9) <= eps);
        REQUIRE(scanner->getMaxNOR() == 1);
        REQUIRE(scanner->getFWFSettings().beamSampleQuality == 3);
        epdiff = scanner->getHeadRelativeEmitterPosition();
        REQUIRE(std::fabs(epdiff[0]) <= eps);
        REQUIRE(std::fabs(epdiff[1]) <= eps);
        REQUIRE(std::fabs(epdiff[2]) <= eps);
        eatt = scanner->getHeadRelativeEmitterAttitude();
        REQUIRE(std::fabs(eatt.getQ0()-0.5) <= eps);
        REQUIRE(std::fabs(eatt.getQ1()+0.5) <= eps);
        REQUIRE(std::fabs(eatt.getQ2()+0.5) <= eps);
        REQUIRE(std::fabs(eatt.getQ3()+0.5) <= eps);
        eraxis = scanner->getScannerHead()->cfg_device_rotateAxis;
        REQUIRE(eraxis[0] == 1);
        REQUIRE(eraxis[1] == 0);
        REQUIRE(eraxis[2] == 0);

        // Livox Mid-100 (MultiScanner)
        scanner = std::static_pointer_cast<Scanner>(
            loader.getAssetById("scanner", "livox-mid-100", nullptr)
        );
        REQUIRE_FALSE(std::dynamic_pointer_cast<SingleScanner>(scanner));
        REQUIRE(std::dynamic_pointer_cast<MultiScanner>(scanner));
        REQUIRE(scanner->getScannerId() == "livox-mid-100");
        REQUIRE(scanner->getDetector(0)->cfg_device_accuracy_m == 0.02);
        REQUIRE(scanner->getDetector(1)->cfg_device_accuracy_m == 0.03);
        REQUIRE(scanner->getDetector(2)->cfg_device_accuracy_m == 0.02);
        REQUIRE(scanner->getBeamDivergence(0) == 0.0027);
        REQUIRE(scanner->getBeamDivergence(1) == 0.0027);
        REQUIRE(scanner->getBeamDivergence(2) == 0.0027);
        REQUIRE(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector(0)));
        REQUIRE(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector(1)));
        REQUIRE_FALSE(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector(2)));
        REQUIRE(std::dynamic_pointer_cast<ConicBeamDeflector>(scanner->getBeamDeflector(2)));
        pfs = scanner->getSupportedPulseFreqs_Hz();
        REQUIRE(pfs.size() == 1);
        REQUIRE(pfs.front() == 50000);
        REQUIRE(scanner->getPulseLength_ns(0) == 4);
        REQUIRE(scanner->getPulseLength_ns(1) == 4);
        REQUIRE(scanner->getPulseLength_ns(2) == 4);
        REQUIRE(scanner->getDetector(0)->cfg_device_rangeMin_m == 2);
        REQUIRE(scanner->getDetector(1)->cfg_device_rangeMin_m == 2);
        REQUIRE(scanner->getDetector(2)->cfg_device_rangeMin_m == 2);
        REQUIRE(std::fabs(scanner->getBeamDeflector(0)->cfg_device_scanAngleMax_rad-0.6108652) <= eps);
        REQUIRE(std::fabs(scanner->getBeamDeflector(1)->cfg_device_scanAngleMax_rad-0.6108652) <= eps);
        REQUIRE(std::fabs(scanner->getBeamDeflector(2)->cfg_device_scanAngleMax_rad-0.6108652) <= eps);
        REQUIRE(std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector(0))->rotorSpeed_rad_1 - 1114.084602) <= eps);
        REQUIRE(std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector(1))->rotorSpeed_rad_1 - 1160.876155) <= eps);
        REQUIRE(std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector(0))->rotorSpeed_rad_2 + 636.6197724) <= eps);
        REQUIRE(std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(scanner->getBeamDeflector(1))->rotorSpeed_rad_2 + 742.298655) <= eps);
        REQUIRE(std::fabs(scanner->getWavelength(0)-905e-9) <= eps);
        REQUIRE(std::fabs(scanner->getWavelength(1)-905e-9) <= eps);
        REQUIRE(std::fabs(scanner->getWavelength(2)-905e-9) <= eps);
        REQUIRE(scanner->getFWFSettings(0).beamSampleQuality == 2);
        REQUIRE(scanner->getFWFSettings(1).beamSampleQuality == 3);
        REQUIRE(scanner->getFWFSettings(2).beamSampleQuality == 4);
        epdiff = scanner->getHeadRelativeEmitterPosition(0)-glm::dvec3(0, 0, 0.1);
        REQUIRE(std::fabs(epdiff[0]) <= eps);
        REQUIRE(std::fabs(epdiff[1]) <= eps);
        REQUIRE(std::fabs(epdiff[2]) <= eps);
        epdiff = scanner->getHeadRelativeEmitterPosition(1)-glm::dvec3(0, 0, -0.1);
        REQUIRE(std::fabs(epdiff[0]) <= eps);
        REQUIRE(std::fabs(epdiff[1]) <= eps);
        REQUIRE(std::fabs(epdiff[2]) <= eps);
        epdiff = scanner->getHeadRelativeEmitterPosition(2);
        REQUIRE(std::fabs(epdiff[0]) <= eps);
        REQUIRE(std::fabs(epdiff[1]) <= eps);
        REQUIRE(std::fabs(epdiff[2]) <= eps);
        eatt = scanner->getHeadRelativeEmitterAttitude(0);
        REQUIRE(std::fabs(eatt.getQ0()-0.965926) <= eps);
        REQUIRE(std::fabs(eatt.getQ1()) <= eps);
        REQUIRE(std::fabs(eatt.getQ2()) <= eps);
        REQUIRE(std::fabs(eatt.getQ3()-0.258819) <= eps);
        eatt = scanner->getHeadRelativeEmitterAttitude(1);
        REQUIRE(std::fabs(eatt.getQ0()-1) <= eps);
        REQUIRE(std::fabs(eatt.getQ1()) <= eps);
        REQUIRE(std::fabs(eatt.getQ2()) <= eps);
        REQUIRE(std::fabs(eatt.getQ3()) <= eps);
        eatt = scanner->getHeadRelativeEmitterAttitude(2);
        REQUIRE(std::fabs(eatt.getQ0()-0.965926) <= eps);
        REQUIRE(std::fabs(eatt.getQ1()) <= eps);
        REQUIRE(std::fabs(eatt.getQ2()) <= eps);
        REQUIRE(std::fabs(eatt.getQ3()+0.258819) <= eps);
        eraxis = scanner->getScannerHead(0)->cfg_device_rotateAxis;
        REQUIRE(eraxis[0] == 1);
        REQUIRE(eraxis[1] == 0);
        REQUIRE(eraxis[2] == 0);
        eraxis = scanner->getScannerHead(1)->cfg_device_rotateAxis;
        REQUIRE(eraxis[0] == 0);
        REQUIRE(eraxis[1] == 1);
        REQUIRE(eraxis[2] == 0);
        eraxis = scanner->getScannerHead(2)->cfg_device_rotateAxis;
        REQUIRE(eraxis[0] == 0);
        REQUIRE(eraxis[1] == 1);
        REQUIRE(eraxis[2] == 0);

        // Unexistent scanner
        REQUIRE_THROWS_AS(
            loader.getAssetById("scanner", "_unexistent_scanner_", nullptr),
            HeliosException
        );
    }

    SECTION("Test Platform Loading") {
        std::string testPlatformsPath = "data/test/test_platforms.xml";
        std::vector<std::string> assetsPath = { "assets/" };
        XmlAssetsLoader loader(testPlatformsPath, assetsPath);

        // Quadrocopter UAV
        std::shared_ptr<Platform> platf = std::dynamic_pointer_cast<Platform>(
            loader.getAssetById("platform", "quadcopter", nullptr)
        );
        std::shared_ptr<HelicopterPlatform> hplatf = std::dynamic_pointer_cast<HelicopterPlatform>(platf);
        REQUIRE(hplatf);
        REQUIRE(platf->name == "Quadrocopter UAV");
        REQUIRE(hplatf->mCfg_drag == 0.0099);
        REQUIRE(hplatf->ef_xy_max == 0.099);
        REQUIRE(hplatf->cfg_speedup_magnitude == 1.99);
        REQUIRE(hplatf->cfg_slowdown_magnitude == 1.99);
        REQUIRE(hplatf->cfg_slowdown_dist_xy == 4.99);
        REQUIRE(std::fabs(hplatf->cfg_pitch_base+0.0959931) <= eps);
        REQUIRE(std::fabs(hplatf->cfg_roll_speed-0.5087635) <= eps);
        REQUIRE(std::fabs(hplatf->cfg_pitch_speed-1.5086626) <= eps);
        REQUIRE(std::fabs(hplatf->cfg_yaw_speed-1.4999360) <= eps);
        REQUIRE(std::fabs(hplatf->cfg_max_roll_offset-0.4433136) <= eps);
        REQUIRE(std::fabs(hplatf->cfg_max_pitch_offset-0.6038839) <= eps);
        glm::dvec3 mpdiff = platf->cfg_device_relativeMountPosition - glm::dvec3(0, 0, 0.21);
        REQUIRE(std::fabs(mpdiff[0]) <= eps);
        REQUIRE(std::fabs(mpdiff[1]) <= eps);
        REQUIRE(std::fabs(mpdiff[2]) <= eps);
        Rotation matt = platf->cfg_device_relativeMountAttitude;
        REQUIRE(std::fabs(matt.getQ0()-0) <= eps);
        REQUIRE(std::fabs(matt.getQ1()-0) <= eps);
        REQUIRE(std::fabs(matt.getQ2()+1) <= eps);
        REQUIRE(std::fabs(matt.getQ3()-0) <= eps);

        // Cirrus SR-22
        platf = std::dynamic_pointer_cast<Platform>(
            loader.getAssetById("platform", "sr22", nullptr)
        );
        REQUIRE(platf->name == "Cirrus SR-22");
        mpdiff = platf->cfg_device_relativeMountPosition - glm::dvec3(0, 0, 0.7);
        REQUIRE(std::fabs(mpdiff[0]) <= eps);
        REQUIRE(std::fabs(mpdiff[1]) <= eps);
        REQUIRE(std::fabs(mpdiff[2]) <= eps);
        matt = platf->cfg_device_relativeMountAttitude;
        REQUIRE(std::fabs(matt.getQ0()-0.5) <= eps);
        REQUIRE(std::fabs(matt.getQ1()-0.5) <= eps);
        REQUIRE(std::fabs(matt.getQ2()-0.5) <= eps);
        REQUIRE(std::fabs(matt.getQ3()+0.5) <= eps);
        REQUIRE(platf->positionXNoiseSource->getClipMin() == 0.0);
        REQUIRE(platf->positionXNoiseSource->getClipMax() == 0.0);
        REQUIRE_FALSE(platf->positionXNoiseSource->isClipEnabled());
        REQUIRE(platf->positionXNoiseSource->getFixedLifespan() == 5);
        std::shared_ptr<NormalNoiseSource<double>> nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(platf->positionXNoiseSource);
        REQUIRE(nns);
        REQUIRE_FALSE(std::dynamic_pointer_cast<UniformNoiseSource<double>>(platf->positionXNoiseSource));
        REQUIRE(nns->getMean() == 0.01);
        REQUIRE(nns->getStdev() == 0.021);
        REQUIRE(platf->positionYNoiseSource->getClipMin() == -0.01);
        REQUIRE(platf->positionYNoiseSource->getClipMax() == 0.0);
        REQUIRE_FALSE(platf->positionYNoiseSource->isClipEnabled());
        REQUIRE(platf->positionYNoiseSource->getFixedLifespan() == 7);
        nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(platf->positionYNoiseSource);
        REQUIRE(nns);
        REQUIRE_FALSE(std::dynamic_pointer_cast<UniformNoiseSource<double>>(platf->positionYNoiseSource));
        REQUIRE(nns->getMean() == -0.01);
        REQUIRE(nns->getStdev() == 0.019);
        REQUIRE(platf->positionZNoiseSource->getClipMin() == -0.03);
        REQUIRE(platf->positionZNoiseSource->getClipMax() == 0.03);
        REQUIRE(platf->positionZNoiseSource->isClipEnabled());
        REQUIRE(platf->positionZNoiseSource->getFixedLifespan() == 1);
        nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(platf->positionZNoiseSource);
        REQUIRE(nns);
        REQUIRE_FALSE(std::dynamic_pointer_cast<UniformNoiseSource<double>>(platf->positionZNoiseSource));
        REQUIRE(nns->getMean() == 0.0);
        REQUIRE(nns->getStdev() == 0.02);
        REQUIRE(platf->attitudeXNoiseSource->getClipMin() == 0.0);
        REQUIRE(platf->attitudeXNoiseSource->getClipMax() == 0.0);
        REQUIRE_FALSE(platf->attitudeXNoiseSource->isClipEnabled());
        REQUIRE(platf->attitudeXNoiseSource->getFixedLifespan() == 1);
        nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(platf->attitudeXNoiseSource);
        REQUIRE(nns);
        REQUIRE_FALSE(std::dynamic_pointer_cast<UniformNoiseSource<double>>(platf->attitudeXNoiseSource));
        REQUIRE(nns->getMean() == 0.0);
        REQUIRE(nns->getStdev() == 0.001);
        REQUIRE(platf->attitudeYNoiseSource->getClipMin() == 0.0);
        REQUIRE(platf->attitudeYNoiseSource->getClipMax() == 0.0);
        REQUIRE_FALSE(platf->attitudeYNoiseSource->isClipEnabled());
        REQUIRE(platf->attitudeYNoiseSource->getFixedLifespan() == 3);
        nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(platf->attitudeYNoiseSource);
        REQUIRE(nns);
        REQUIRE_FALSE(std::dynamic_pointer_cast<UniformNoiseSource<double>>(platf->attitudeYNoiseSource));
        REQUIRE(nns->getMean() == 0.0);
        REQUIRE(nns->getStdev() == 0.001);
        REQUIRE(platf->attitudeZNoiseSource->getClipMin() == 0.0);
        REQUIRE(platf->attitudeZNoiseSource->getClipMax() == 0.0);
        REQUIRE_FALSE(platf->attitudeZNoiseSource->isClipEnabled());
        REQUIRE(platf->attitudeZNoiseSource->getFixedLifespan() == 11);
        nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(platf->attitudeZNoiseSource);
        REQUIRE(nns);
        REQUIRE_FALSE(std::dynamic_pointer_cast<UniformNoiseSource<double>>(platf->attitudeZNoiseSource));
        REQUIRE(nns->getMean() == 0.0);
        REQUIRE(nns->getStdev() == 0.001);

        // Tractor
        platf = std::dynamic_pointer_cast<Platform>(
            loader.getAssetById("platform", "tractor")
        );
        REQUIRE(platf->name == "Tractor");
        mpdiff = platf->cfg_device_relativeMountPosition - glm::dvec3(0, 1, 4);
        REQUIRE(std::fabs(mpdiff[0]) <= eps);
        REQUIRE(std::fabs(mpdiff[1]) <= eps);
        REQUIRE(std::fabs(mpdiff[2]) <= eps);
        matt = platf->cfg_device_relativeMountAttitude;
        REQUIRE(std::fabs(matt.getQ0()-0.6830127) <= eps);
        REQUIRE(std::fabs(matt.getQ1()+0.1830127) <= eps);
        REQUIRE(std::fabs(matt.getQ2()-0.1830127) <= eps);
        REQUIRE(std::fabs(matt.getQ3()+0.6830127) <= eps);
        std::shared_ptr<GroundVehiclePlatform> gplatf = std::dynamic_pointer_cast<GroundVehiclePlatform>(platf);
        REQUIRE(gplatf);
        REQUIRE(gplatf->mCfg_drag == 0.00499);

        // Tripod
        platf = std::dynamic_pointer_cast<Platform>(
            loader.getAssetById("platform", "tripod", nullptr)
        );
        REQUIRE(platf->name == "TLS Tripod");
        mpdiff = platf->cfg_device_relativeMountPosition - glm::dvec3(0, 0, 1.5);
        REQUIRE(std::fabs(mpdiff[0]) <= eps);
        REQUIRE(std::fabs(mpdiff[1]) <= eps);
        REQUIRE(std::fabs(mpdiff[2]) <= eps);
    }
}