#pragma once

#include <BaseTest.h>
#include <assetloading/XmlUtils.h>
#include <assetloading/XmlSceneLoader.h>
#include <assetloading/XmlAssetsLoader.h>
#include <assetloading/XmlSurveyLoader.h>


namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Asset loading tests
 */
class AssetLoadingTest : public BaseTest{
public:
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
    // TODO Rethink : Implement
    std::string testScannersPath = "test/test_scanners.xml";
    std::string assetsPath = "assets/";
    XmlAssetsLoader loader(testScannersPath, assetsPath);
    std::shared_ptr<Scanner> scanner = std::static_pointer_cast<Scanner>(
        loader.getAssetById("scanner", "scanner", nullptr)
    );
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