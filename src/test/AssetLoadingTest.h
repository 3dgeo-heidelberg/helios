#pragma once

#include <BaseTest.h>
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


namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Asset loading tests
 */
class AssetLoadingTest : public BaseTest{
public:
    // ***  ATTRIBUTE  *** //
    // ******************* //
    /**
     * @brief Decimal precision tolerance
     */
    double const eps = 0.00001;

    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief Asset loading test constructor
     */
    AssetLoadingTest() : BaseTest("Asset loading test"){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

    // ***  SUB-TESTS  *** //
    // ******************* //
    /**
     * @brief Test scanner loading
     * @return True if passed, false otherwise
     */
    bool testScannerLoading();
    /**
     * @brief Test scanner settings loading
     * @return True if passed, false otherwise
     */
    bool testScannerSettingsLoading();
    /**
     * @brief Test full waveform settings loading
     * @return True if passed, false otherwise
     */
    bool testFWFSettingsLoading();
    /**
     * @brief Test platform loading
     * @return True if passed, false otherwise
     */
    bool testPlatformLoading();
    /**
     * @brief Test platform settings loading
     * @return True if passed, false otherwise
     */
    bool testPlatformSettingsLoading();
    /**
     * @brief Test Wavefront OBJ loading
     * @return True if passed, false otherwise
     */
    bool testObjLoading();
    /**
     * @brief Test voxel loading
     * @return True if passed, false otherwise
     */
    bool testVoxelLoading();
    /**
     * @brief Test detailed voxel loading
     * @return True if passed, false otherwise
     */
    bool testDetailedVoxelLoading();
    /**
     * @brief Test TIFF loading
     * @return True if passed, false otherwise
     */
    bool testTiffLoading();
    /**
     * @brief Test scene loading
     * @return True if passed, false otherwise
     */
    bool testSceneLoading();
    /**
     * @brief Test survey loading
     * @return True if passed, false otherwise
     */
    bool testSurveyLoading();
};


// ***  R U N  *** //
// *************** //
bool AssetLoadingTest::run(){
    // Run tests
    try{
        if(!testScannerLoading()) return false;
        if(!testScannerSettingsLoading()) return false;
        if(!testFWFSettingsLoading()) return false;
        if(!testPlatformLoading()) return false;
        if(!testPlatformSettingsLoading()) return false;
        if(!testObjLoading()) return false;
        if(!testVoxelLoading()) return false;
        if(!testDetailedVoxelLoading()) return false;
        if(!testTiffLoading()) return false;
        if(!testSceneLoading()) return false;
        if(!testSurveyLoading()) return false;
    }
    catch(std::exception &ex){
        return false;
    }
    return true;
}


// ***  SUB-TESTS  *** //
// ******************* //
bool AssetLoadingTest::testScannerLoading(){
    // Prepare scanner loading
    std::string testScannersPath = "data/test/test_scanners.xml";
    std::string assetsPath = "assets/";
    XmlAssetsLoader loader(testScannersPath, assetsPath);
    // Load and validate Leica ALS50
    std::shared_ptr<Scanner> scanner = std::static_pointer_cast<Scanner>(
        loader.getAssetById("scanner", "leica_als50", nullptr)
    );
    // TODO Rethink: Apply Single/Multi test to all cases
    if(std::dynamic_pointer_cast<SingleScanner>(scanner)==nullptr)
        return false;
    /*if(std::dynamic_pointer_cast<MultiScanner>(scanner)!=nullptr)
        return false;*/
    if(scanner->getScannerId() != "leica_als50") return false;
    if(scanner->getDetector()->cfg_device_accuracy_m != 0.05) return false;
    if(scanner->getBeamDivergence() != 0.00033) return false;
    if(scanner->name != "Leica ALS50") return false;
    if(
        std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(
            scanner->getBeamDeflector()
        ) == nullptr
    ) return false;
    std::list<int> &pfs = scanner->getSupportedPulseFreqs_Hz();
    if(pfs.size() != 1) return false;
    if(pfs.front() != 83000) return false;
    if(scanner->getPulseFreq_Hz() != 83000) return false;
    if(scanner->getPulseLength_ns() != 10) return false;
    if(scanner->getDetector()->cfg_device_rangeMin_m != 200) return false;
    if( std::fabs(
            scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.6544985
        ) > eps
    ) return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz != 25)
        return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz != 70)
        return false;
    // Load and validate Leica ALST50-II
    scanner = std::static_pointer_cast<Scanner>(
        loader.getAssetById("scanner", "leica_als50-ii", nullptr)
    );
    if(scanner->getScannerId() != "leica_als50-ii") return false;
    if(scanner->getDetector()->cfg_device_accuracy_m != 0.05) return false;
    if(scanner->getBeamDivergence() != 0.00022) return false;
    if(scanner->name != "Leica ALS50-II") return false;
    if(
        std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(
            scanner->getBeamDeflector()
        ) == nullptr
    ) return false;
    pfs = scanner->getSupportedPulseFreqs_Hz();
    if(pfs.size() != 3) return false;
    std::list<int>::iterator pfsit = pfs.begin();
    if(*pfsit != 20000) return false;
    ++pfsit;
    if(*pfsit != 60000) return false;
    ++pfsit;
    if(*pfsit != 150000) return false;
    if(scanner->getPulseFreq_Hz() != 20000) return false;
    if(scanner->getPulseLength_ns() != 10) return false;
    if(scanner->getDetector()->cfg_device_rangeMin_m != 200) return false;
    if( std::fabs(
            scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.6544985
        ) > eps
    ) return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz != 0)
        return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz != 90)
        return false;
    if(scanner->getMaxNOR() != 4) return false;
    // Load and validate Optech 2033
    scanner = std::static_pointer_cast<Scanner>(
        loader.getAssetById("scanner", "optech_2033", nullptr)
    );
    if(scanner->getScannerId() != "optech_2033") return false;
    if(scanner->getDetector()->cfg_device_accuracy_m != 0.01) return false;
    if(scanner->getBeamDivergence() != 0.000424) return false;
    if(scanner->name != "Optech ALTM 2033") return false;
    if(
        std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(
            scanner->getBeamDeflector()
        ) == nullptr
    ) return false;
    pfs = scanner->getSupportedPulseFreqs_Hz();
    if(pfs.size() != 1) return false;
    if(pfs.front() != 33000) return false;
    if(scanner->getPulseLength_ns() != 8) return false;
    if(scanner->getDetector()->cfg_device_rangeMin_m != 1) return false;
    if( std::fabs(
            scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.3490659
        ) > eps
    ) return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz!=0) return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz!=30)
        return false;
    if(
        std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(
            scanner->getBeamDeflector()
        )->cfg_device_scanProduct != 590
    ) return false;
    if(scanner->getWavelength() != 1047e-9) return false;
    glm::dvec3 epdiff = scanner->getHeadRelativeEmitterPosition()
        - glm::dvec3(0, 0.085, 0.06);
    if(
        std::fabs(epdiff[0]) > eps ||
        std::fabs(epdiff[1]) > eps ||
        std::fabs(epdiff[2]) > eps
    ) return false;
    Rotation eatt = scanner->getHeadRelativeEmitterAttitude();
    if(
        eatt.getQ0()!=1 || eatt.getQ1()!=0 ||
        eatt.getQ2()!=0 || eatt.getQ3()!=0
    ) return false;
    glm::dvec3 eraxis = scanner->getScannerHead()->cfg_device_rotateAxis;
    if(eraxis[0] != 0 || eraxis[1] != 0 || eraxis[2] != 1) return false;
    // Load and validate Riegl LMS-Q560
    scanner = std::static_pointer_cast<Scanner>(
        loader.getAssetById("scanner", "riegl_lms-q560")
    );
    if(scanner->getScannerId() != "riegl_lms-q560") return false;
    if(scanner->getDetector()->cfg_device_accuracy_m != 0.02) return false;
    if(scanner->getBeamDivergence() != 0.0005) return false;
    if(scanner->name != "RIEGL LMS-Q560") return false;
    if(
        std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(
            scanner->getBeamDeflector()
        ) == nullptr
    ) return false;
    pfs = scanner->getSupportedPulseFreqs_Hz();
    if(pfs.size() != 5) return false;
    pfsit = pfs.begin();
    if(*pfsit != 50000){return false;}   ++pfsit;
    if(*pfsit != 100000){return false;}  ++pfsit;
    if(*pfsit != 180000){return false;}  ++pfsit;
    if(*pfsit != 200000){return false;}  ++pfsit;
    if(*pfsit != 240000){return false;}  ++pfsit;
    if(scanner->getPulseLength_ns() != 4) return false;
    if( std::fabs(
            scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.7853982
        ) > eps
    ) return false;
    if( std::fabs(
            std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(
                scanner->getBeamDeflector()
            )->getScanAngleEffectiveMax_rad() - 0.3926991
        ) > eps
    ) return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz != 10)
        return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz != 160)
        return false;
    // Load and validate RIEGLS VQ-880-G
    scanner = std::static_pointer_cast<Scanner>(
        loader.getAssetById("scanner", "riegl_vq-880g", nullptr)
    );
    if(scanner->getScannerId() != "riegl_vq-880g") return false;
    if(scanner->getDetector()->cfg_device_accuracy_m != 0.025) return false;
    if(scanner->getBeamDivergence() != 0.0003) return false;
    if(scanner->name != "RIEGL VQ-880-G") return false;
    if(
        std::dynamic_pointer_cast<ConicBeamDeflector>(
            scanner->getBeamDeflector()
        ) == nullptr
    ) return false;
    pfs = scanner->getSupportedPulseFreqs_Hz();
    if(pfs.size()!=4) return false;
    pfsit = pfs.begin();
    if(*pfsit != 150000){return false;} ++pfsit;
    if(*pfsit != 300000){return false;} ++pfsit;
    if(*pfsit != 600000){return false;} ++pfsit;
    if(*pfsit != 900000){return false;} ++pfsit;
    if(scanner->getPulseLength_ns() != 2) return false;
    if( std::fabs(
            scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.3490659
        ) > eps
    ) return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz != 28)
        return false;
    if(scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz != 200)
        return false;
    if(std::fabs(scanner->getWavelength()-1064e-9) > eps) return false;
    epdiff = scanner->getHeadRelativeEmitterPosition()
        - glm::dvec3(0, 0.085, 0.06);
    if(
        std::fabs(epdiff[0]) > eps ||
        std::fabs(epdiff[1]) > eps ||
        std::fabs(epdiff[2]) > eps
    ) return false;
    eatt = scanner->getHeadRelativeEmitterAttitude();
    if(
        eatt.getQ0()!=1 || eatt.getQ1()!=0 ||
        eatt.getQ2()!=0 || eatt.getQ3()!=0
    ) return false;
    eraxis = scanner->getScannerHead()->cfg_device_rotateAxis;
    if(eraxis[0] != 0 || eraxis[1] != 0 || eraxis[2] != 1) return false;
    // Load and validate LIVOX Mid-70
    scanner = std::static_pointer_cast<Scanner>(
        loader.getAssetById("scanner", "livox_mid-70", nullptr)
    );
    if(scanner->getScannerId() != "livox_mid-70") return false;
    if(scanner->getDetector()->cfg_device_accuracy_m != 0.02) return false;
    if(scanner->getBeamDivergence() != 0.004887) return false;
    if(scanner->name != "Livox Mid-70") return false;
    if(
        std::dynamic_pointer_cast<RisleyBeamDeflector>(
            scanner->getBeamDeflector()
        ) == nullptr
    ) return false;
    pfs = scanner->getSupportedPulseFreqs_Hz();
    if(pfs.size() != 1) return false;
    if(pfs.front() != 100000) return false;
    if(scanner->getPulseLength_ns() != 4) return false;
    if(scanner->getDetector()->cfg_device_rangeMin_m != 2) return false;
    if( std::fabs(
            scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad-0.6108652
        ) > eps
    ) return false;
    if( std::fabs(
            std::dynamic_pointer_cast<RisleyBeamDeflector>(
                scanner->getBeamDeflector()
            )->rotorSpeed_rad_1 - 1160.876155  // Hz / (2 pi)
        ) > eps
    ) return false;
    if( std::fabs(
            std::dynamic_pointer_cast<RisleyBeamDeflector>(
                scanner->getBeamDeflector()
            )->rotorSpeed_rad_2 + 742.298655   // Hz / (2 pi)
        ) > eps
    ) return false;
    if(std::fabs(scanner->getWavelength()-905e-9) > eps) return false;
    if(scanner->getMaxNOR() != 1) return false;
    if(scanner->getFWFSettings().beamSampleQuality != 3) return false;
    epdiff = scanner->getHeadRelativeEmitterPosition();
    if(
        std::fabs(epdiff[0]) > eps ||
        std::fabs(epdiff[1]) > eps ||
        std::fabs(epdiff[2]) > eps
    ) return false;
    eatt = scanner->getHeadRelativeEmitterAttitude();
    if(
        std::fabs(eatt.getQ0()-0.5)>eps || std::fabs(eatt.getQ1()+0.5)>eps ||
        std::fabs(eatt.getQ2()+0.5)>eps || std::fabs(eatt.getQ3()+0.5)>eps
    ) return false;
    eraxis = scanner->getScannerHead()->cfg_device_rotateAxis;
        - glm::dvec3(0, 0, 0);
    if(eraxis[0] != 1 || eraxis[1] != 0 || eraxis[2] != 0) return false;
    // Load and validate LIVOX Mid-100 with triple channel scanner
    // TODO Rethink : Implement LIVOX Mid-100 with triple channel test
    // Load and validate unexistent scanner
    try{
        scanner = std::dynamic_pointer_cast<Scanner>(loader.getAssetById(
            "scanner", "_unexistent_scanner_", nullptr
        ));
        return false; // Must not be reached, exception before
    }
    catch(HeliosException &hex){}
    return true;
}

bool AssetLoadingTest::testScannerSettingsLoading(){
    // TODO Rethink : Implement
    return true;
}

bool AssetLoadingTest::testFWFSettingsLoading(){
    // TODO Rethink : Implement
    return true;
}

bool AssetLoadingTest::testPlatformLoading(){
    // Prepare platform loading
    std::string testPlatformsPath = "data/test/test_scanners.xml";
    std::string assetsPath = "assets/";
    XmlAssetsLoader loader(testPlatformsPath, assetsPath);
    // Load and validate Quadropcopter UAV
    // Load and validate Cirrus SR-22
    // Load and validate Tractor
    // Load and validate Tripod
    // TODO Rethink : Implement
    return true;
}

bool AssetLoadingTest::testPlatformSettingsLoading(){
    // TODO Rethink : Implement
    return true;
}

bool AssetLoadingTest::testObjLoading(){
    // TODO Rethink : Implement
    return true;
}

bool AssetLoadingTest::testVoxelLoading(){
    // TODO Rethink : Implement
    return true;
}

bool AssetLoadingTest::testDetailedVoxelLoading(){
    // TODO Rethink : Implement
    return true;
}

bool AssetLoadingTest::testTiffLoading(){
    // TODO Rethink : Implement
    return true;
}

bool AssetLoadingTest::testSceneLoading(){
    // TODO Rethink : Implement
    return true;
}

bool AssetLoadingTest::testSurveyLoading(){
    // TODO Rethink : Implement
    return true;
}


}