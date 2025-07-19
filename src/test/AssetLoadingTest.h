#pragma once

#include <BaseTest.h>
#include <assetloading/XmlAssetsLoader.h>
#include <assetloading/XmlSceneLoader.h>
#include <assetloading/XmlSurveyLoader.h>
#include <assetloading/XmlUtils.h>
#include <noise/NoiseSource.h>
#include <noise/NormalNoiseSource.h>
#include <noise/UniformNoiseSource.h>
#include <platform/GroundVehiclePlatform.h>
#include <platform/HelicopterPlatform.h>
#include <platform/LinearPathPlatform.h>
#include <platform/Platform.h>
#include <scanner/MultiScanner.h>
#include <scanner/SingleScanner.h>
#include <scanner/beamDeflector/ConicBeamDeflector.h>
#include <scanner/beamDeflector/OscillatingMirrorBeamDeflector.h>
#include <scanner/beamDeflector/PolygonMirrorBeamDeflector.h>
#include <scanner/beamDeflector/RisleyBeamDeflector.h>

namespace HeliosTests {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Asset loading tests
 */
class AssetLoadingTest : public BaseTest
{
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
  AssetLoadingTest()
    : BaseTest("Asset loading test")
  {
  }

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
   * @brief Test platform loading
   * @return True if passed, false otherwise
   */
  bool testPlatformLoading();
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
bool
AssetLoadingTest::run()
{
  // Run tests
  try {
    if (!testScannerLoading())
      return false;
    if (!testPlatformLoading())
      return false;
    if (!testObjLoading())
      return false;
    if (!testVoxelLoading())
      return false;
    if (!testDetailedVoxelLoading())
      return false;
    if (!testTiffLoading())
      return false;
    if (!testSceneLoading())
      return false;
    if (!testSurveyLoading())
      return false;
  } catch (std::exception& ex) {
    return false;
  }
  return true;
}

// ***  SUB-TESTS  *** //
// ******************* //
bool
AssetLoadingTest::testScannerLoading()
{
  // Prepare scanner loading
  std::string testScannersPath = "data/test/test_scanners.xml";
  std::vector<std::string> assetsPath = { "assets/" };
  XmlAssetsLoader loader(testScannersPath, assetsPath);
  // Load and validate Leica ALS50
  std::shared_ptr<Scanner> scanner = std::static_pointer_cast<Scanner>(
    loader.getAssetById("scanner", "leica_als50", nullptr));
  if (std::dynamic_pointer_cast<SingleScanner>(scanner) == nullptr)
    return false;
  if (std::dynamic_pointer_cast<MultiScanner>(scanner) != nullptr)
    return false;
  if (scanner->getScannerId() != "leica_als50")
    return false;
  if (scanner->getDetector()->cfg_device_accuracy_m != 0.05)
    return false;
  if (scanner->getBeamDivergence() != 0.00033)
    return false;
  if (scanner->name != "Leica ALS50")
    return false;
  if (std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(
        scanner->getBeamDeflector()) == nullptr)
    return false;
  std::list<int>& pfs = scanner->getSupportedPulseFreqs_Hz();
  if (pfs.size() != 1)
    return false;
  if (pfs.front() != 83000)
    return false;
  if (scanner->getPulseFreq_Hz() != 83000)
    return false;
  if (scanner->getPulseLength_ns() != 10)
    return false;
  if (scanner->getDetector()->cfg_device_rangeMin_m != 200)
    return false;
  if (std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad -
                0.6544985) > eps)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz != 25)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz != 70)
    return false;
  // Load and validate Leica ALST50-II
  scanner = std::static_pointer_cast<Scanner>(
    loader.getAssetById("scanner", "leica_als50-ii", nullptr));
  if (std::dynamic_pointer_cast<SingleScanner>(scanner) == nullptr)
    return false;
  if (std::dynamic_pointer_cast<MultiScanner>(scanner) != nullptr)
    return false;
  if (scanner->getScannerId() != "leica_als50-ii")
    return false;
  if (scanner->getDetector()->cfg_device_accuracy_m != 0.05)
    return false;
  if (scanner->getBeamDivergence() != 0.00022)
    return false;
  if (scanner->name != "Leica ALS50-II")
    return false;
  if (std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(
        scanner->getBeamDeflector()) == nullptr)
    return false;
  pfs = scanner->getSupportedPulseFreqs_Hz();
  if (pfs.size() != 3)
    return false;
  std::list<int>::iterator pfsit = pfs.begin();
  if (*pfsit != 20000)
    return false;
  ++pfsit;
  if (*pfsit != 60000)
    return false;
  ++pfsit;
  if (*pfsit != 150000)
    return false;
  if (scanner->getPulseFreq_Hz() != 20000)
    return false;
  if (scanner->getPulseLength_ns() != 10)
    return false;
  if (scanner->getDetector()->cfg_device_rangeMin_m != 200)
    return false;
  if (std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad -
                0.6544985) > eps)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz != 0)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz != 90)
    return false;
  if (scanner->getMaxNOR() != 4)
    return false;
  // Load and validate Optech 2033
  scanner = std::static_pointer_cast<Scanner>(
    loader.getAssetById("scanner", "optech_2033", nullptr));
  if (std::dynamic_pointer_cast<SingleScanner>(scanner) == nullptr)
    return false;
  if (std::dynamic_pointer_cast<MultiScanner>(scanner) != nullptr)
    return false;
  if (scanner->getScannerId() != "optech_2033")
    return false;
  if (scanner->getDetector()->cfg_device_accuracy_m != 0.01)
    return false;
  if (scanner->getBeamDivergence() != 0.000424)
    return false;
  if (scanner->name != "Optech ALTM 2033")
    return false;
  if (std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(
        scanner->getBeamDeflector()) == nullptr)
    return false;
  pfs = scanner->getSupportedPulseFreqs_Hz();
  if (pfs.size() != 1)
    return false;
  if (pfs.front() != 33000)
    return false;
  if (scanner->getPulseLength_ns() != 8)
    return false;
  if (scanner->getDetector()->cfg_device_rangeMin_m != 1)
    return false;
  if (std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad -
                0.3490659) > eps)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz != 0)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz != 30)
    return false;
  if (std::dynamic_pointer_cast<OscillatingMirrorBeamDeflector>(
        scanner->getBeamDeflector())
        ->cfg_device_scanProduct != 590)
    return false;
  if (scanner->getWavelength() != 1047e-9)
    return false;
  glm::dvec3 epdiff =
    scanner->getHeadRelativeEmitterPosition() - glm::dvec3(0, 0.085, 0.06);
  if (std::fabs(epdiff[0]) > eps || std::fabs(epdiff[1]) > eps ||
      std::fabs(epdiff[2]) > eps)
    return false;
  Rotation eatt = scanner->getHeadRelativeEmitterAttitude();
  if (eatt.getQ0() != 1 || eatt.getQ1() != 0 || eatt.getQ2() != 0 ||
      eatt.getQ3() != 0)
    return false;
  glm::dvec3 eraxis = scanner->getScannerHead()->cfg_device_rotateAxis;
  if (eraxis[0] != 0 || eraxis[1] != 0 || eraxis[2] != 1)
    return false;
  // Load and validate Riegl LMS-Q560
  scanner = std::static_pointer_cast<Scanner>(
    loader.getAssetById("scanner", "riegl_lms-q560"));
  if (std::dynamic_pointer_cast<SingleScanner>(scanner) == nullptr)
    return false;
  if (std::dynamic_pointer_cast<MultiScanner>(scanner) != nullptr)
    return false;
  if (scanner->getScannerId() != "riegl_lms-q560")
    return false;
  if (scanner->getDetector()->cfg_device_accuracy_m != 0.02)
    return false;
  if (scanner->getBeamDivergence() != 0.0005)
    return false;
  if (scanner->name != "RIEGL LMS-Q560")
    return false;
  if (std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(
        scanner->getBeamDeflector()) == nullptr)
    return false;
  pfs = scanner->getSupportedPulseFreqs_Hz();
  if (pfs.size() != 5)
    return false;
  pfsit = pfs.begin();
  if (*pfsit != 50000) {
    return false;
  }
  ++pfsit;
  if (*pfsit != 100000) {
    return false;
  }
  ++pfsit;
  if (*pfsit != 180000) {
    return false;
  }
  ++pfsit;
  if (*pfsit != 200000) {
    return false;
  }
  ++pfsit;
  if (*pfsit != 240000) {
    return false;
  }
  ++pfsit;
  if (scanner->getPulseLength_ns() != 4)
    return false;
  if (std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad -
                0.7853982) > eps)
    return false;
  if (std::fabs(std::dynamic_pointer_cast<PolygonMirrorBeamDeflector>(
                  scanner->getBeamDeflector())
                  ->getScanAngleEffectiveMax_rad() -
                0.3926991) > eps)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz != 10)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz != 160)
    return false;
  // Load and validate RIEGLS VQ-880-G
  scanner = std::static_pointer_cast<Scanner>(
    loader.getAssetById("scanner", "riegl_vq-880g", nullptr));
  if (std::dynamic_pointer_cast<SingleScanner>(scanner) == nullptr)
    return false;
  if (std::dynamic_pointer_cast<MultiScanner>(scanner) != nullptr)
    return false;
  if (scanner->getScannerId() != "riegl_vq-880g")
    return false;
  if (scanner->getDetector()->cfg_device_accuracy_m != 0.025)
    return false;
  if (scanner->getBeamDivergence() != 0.0003)
    return false;
  if (scanner->name != "RIEGL VQ-880-G")
    return false;
  if (std::dynamic_pointer_cast<ConicBeamDeflector>(
        scanner->getBeamDeflector()) == nullptr)
    return false;
  pfs = scanner->getSupportedPulseFreqs_Hz();
  if (pfs.size() != 4)
    return false;
  pfsit = pfs.begin();
  if (*pfsit != 150000) {
    return false;
  }
  ++pfsit;
  if (*pfsit != 300000) {
    return false;
  }
  ++pfsit;
  if (*pfsit != 600000) {
    return false;
  }
  ++pfsit;
  if (*pfsit != 900000) {
    return false;
  }
  ++pfsit;
  if (scanner->getPulseLength_ns() != 2)
    return false;
  if (std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad -
                0.3490659) > eps)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz != 28)
    return false;
  if (scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz != 200)
    return false;
  if (std::fabs(scanner->getWavelength() - 1064e-9) > eps)
    return false;
  epdiff =
    scanner->getHeadRelativeEmitterPosition() - glm::dvec3(0, 0.085, 0.06);
  if (std::fabs(epdiff[0]) > eps || std::fabs(epdiff[1]) > eps ||
      std::fabs(epdiff[2]) > eps)
    return false;
  eatt = scanner->getHeadRelativeEmitterAttitude();
  if (eatt.getQ0() != 1 || eatt.getQ1() != 0 || eatt.getQ2() != 0 ||
      eatt.getQ3() != 0)
    return false;
  eraxis = scanner->getScannerHead()->cfg_device_rotateAxis;
  if (eraxis[0] != 0 || eraxis[1] != 0 || eraxis[2] != 1)
    return false;
  // Load and validate LIVOX Mid-70
  scanner = std::static_pointer_cast<Scanner>(
    loader.getAssetById("scanner", "livox_mid-70", nullptr));
  if (std::dynamic_pointer_cast<SingleScanner>(scanner) == nullptr)
    return false;
  if (std::dynamic_pointer_cast<MultiScanner>(scanner) != nullptr)
    return false;
  if (scanner->getScannerId() != "livox_mid-70")
    return false;
  if (scanner->getDetector()->cfg_device_accuracy_m != 0.02)
    return false;
  if (scanner->getBeamDivergence() != 0.004887)
    return false;
  if (scanner->name != "Livox Mid-70")
    return false;
  if (std::dynamic_pointer_cast<RisleyBeamDeflector>(
        scanner->getBeamDeflector()) == nullptr)
    return false;
  pfs = scanner->getSupportedPulseFreqs_Hz();
  if (pfs.size() != 1)
    return false;
  if (pfs.front() != 100000)
    return false;
  if (scanner->getPulseLength_ns() != 4)
    return false;
  if (scanner->getDetector()->cfg_device_rangeMin_m != 2)
    return false;
  if (std::fabs(scanner->getBeamDeflector()->cfg_device_scanAngleMax_rad -
                0.6108652) > eps)
    return false;
  if (std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(
                  scanner->getBeamDeflector())
                  ->prisms[0]
                  .rotation_speed_rad -
                1160.876155 // Hz / (2 pi)
                ) > eps)
    return false;
  if (std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(
                  scanner->getBeamDeflector())
                  ->prisms[1]
                  .rotation_speed_rad +
                742.298655 // Hz / (2 pi)
                ) > eps)
    return false;
  if (std::fabs(scanner->getWavelength() - 905e-9) > eps)
    return false;
  if (scanner->getMaxNOR() != 1)
    return false;
  if (scanner->getFWFSettings().beamSampleQuality != 3)
    return false;
  epdiff = scanner->getHeadRelativeEmitterPosition();
  if (std::fabs(epdiff[0]) > eps || std::fabs(epdiff[1]) > eps ||
      std::fabs(epdiff[2]) > eps)
    return false;
  eatt = scanner->getHeadRelativeEmitterAttitude();
  if (std::fabs(eatt.getQ0() - 0.5) > eps ||
      std::fabs(eatt.getQ1() + 0.5) > eps ||
      std::fabs(eatt.getQ2() + 0.5) > eps ||
      std::fabs(eatt.getQ3() + 0.5) > eps)
    return false;
  eraxis = scanner->getScannerHead()->cfg_device_rotateAxis;
  if (eraxis[0] != 1 || eraxis[1] != 0 || eraxis[2] != 0)
    return false;
  // Load and validate LIVOX Mid-100 with triple channel scanner
  scanner = std::static_pointer_cast<Scanner>(
    loader.getAssetById("scanner", "livox-mid-100", nullptr));
  if (std::dynamic_pointer_cast<SingleScanner>(scanner) != nullptr)
    return false;
  if (std::dynamic_pointer_cast<MultiScanner>(scanner) == nullptr)
    return false;
  if (scanner->getScannerId() != "livox-mid-100")
    return false;
  if (scanner->getDetector(0)->cfg_device_accuracy_m != 0.02)
    return false;
  if (scanner->getDetector(1)->cfg_device_accuracy_m != 0.03)
    return false;
  if (scanner->getDetector(2)->cfg_device_accuracy_m != 0.02)
    return false;
  if (scanner->getBeamDivergence(0) != 0.0027)
    return false;
  if (scanner->getBeamDivergence(1) != 0.0027)
    return false;
  if (scanner->getBeamDivergence(2) != 0.0027)
    return false;
  if (std::dynamic_pointer_cast<RisleyBeamDeflector>(
        scanner->getBeamDeflector(0)) == nullptr)
    return false;
  if (std::dynamic_pointer_cast<RisleyBeamDeflector>(
        scanner->getBeamDeflector(1)) == nullptr)
    return false;
  if (std::dynamic_pointer_cast<RisleyBeamDeflector>(
        scanner->getBeamDeflector(2)) != nullptr)
    return false;
  if (std::dynamic_pointer_cast<ConicBeamDeflector>(
        scanner->getBeamDeflector(2)) == nullptr)
    return false;
  pfs = scanner->getSupportedPulseFreqs_Hz();
  if (pfs.size() != 1)
    return false;
  if (pfs.front() != 50000)
    return false;
  if (scanner->getPulseLength_ns(0) != 4)
    return false;
  if (scanner->getPulseLength_ns(1) != 4)
    return false;
  if (scanner->getPulseLength_ns(2) != 4)
    return false;
  if (scanner->getDetector(0)->cfg_device_rangeMin_m != 2)
    return false;
  if (scanner->getDetector(1)->cfg_device_rangeMin_m != 2)
    return false;
  if (scanner->getDetector(2)->cfg_device_rangeMin_m != 2)
    return false;
  if (std::fabs(scanner->getBeamDeflector(0)->cfg_device_scanAngleMax_rad -
                0.6108652) > eps)
    return false;
  if (std::fabs(scanner->getBeamDeflector(1)->cfg_device_scanAngleMax_rad -
                0.6108652) > eps)
    return false;
  if (std::fabs(scanner->getBeamDeflector(2)->cfg_device_scanAngleMax_rad -
                0.6108652) > eps)
    return false;
  if (std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(
                  scanner->getBeamDeflector(0))
                  ->prisms[0]
                  .rotation_speed_rad -
                1114.084602) > eps)
    return false;
  if (std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(
                  scanner->getBeamDeflector(1))
                  ->prisms[0]
                  .rotation_speed_rad -
                1160.876155) > eps)
    return false;
  if (std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(
                  scanner->getBeamDeflector(0))
                  ->prisms[1]
                  .rotation_speed_rad +
                636.6197724) > eps)
    return false;
  if (std::fabs(std::dynamic_pointer_cast<RisleyBeamDeflector>(
                  scanner->getBeamDeflector(1))
                  ->prisms[1]
                  .rotation_speed_rad +
                742.298655) > eps)
    return false;
  if (std::fabs(scanner->getWavelength(0) - 905e-9) > eps)
    return false;
  if (std::fabs(scanner->getWavelength(1) - 905e-9) > eps)
    return false;
  if (std::fabs(scanner->getWavelength(2) - 905e-9) > eps)
    return false;
  if (scanner->getFWFSettings(0).beamSampleQuality != 2)
    return false;
  if (scanner->getFWFSettings(1).beamSampleQuality != 3)
    return false;
  if (scanner->getFWFSettings(2).beamSampleQuality != 4)
    return false;
  epdiff = scanner->getHeadRelativeEmitterPosition(0) - glm::dvec3(0, 0, 0.1);
  if (std::fabs(epdiff[0]) > eps || std::fabs(epdiff[1]) > eps ||
      std::fabs(epdiff[2]) > eps)
    return false;
  epdiff = scanner->getHeadRelativeEmitterPosition(1) - glm::dvec3(0, 0, -0.1);
  if (std::fabs(epdiff[0]) > eps || std::fabs(epdiff[1]) > eps ||
      std::fabs(epdiff[2]) > eps)
    return false;
  epdiff = scanner->getHeadRelativeEmitterPosition(2);
  if (std::fabs(epdiff[0]) > eps || std::fabs(epdiff[1]) > eps ||
      std::fabs(epdiff[2]) > eps)
    return false;
  eatt = scanner->getHeadRelativeEmitterAttitude(0);
  if (std::fabs(eatt.getQ0() - 0.965926) > eps ||
      std::fabs(eatt.getQ1()) > eps || std::fabs(eatt.getQ2()) > eps ||
      std::fabs(eatt.getQ3() - 0.258819) > eps)
    return false;
  eatt = scanner->getHeadRelativeEmitterAttitude(1);
  if (std::fabs(eatt.getQ0() - 1) > eps || std::fabs(eatt.getQ1()) > eps ||
      std::fabs(eatt.getQ2()) > eps || std::fabs(eatt.getQ3()) > eps)
    return false;
  eatt = scanner->getHeadRelativeEmitterAttitude(2);
  if (std::fabs(eatt.getQ0() - 0.965926) > eps ||
      std::fabs(eatt.getQ1()) > eps || std::fabs(eatt.getQ2()) > eps ||
      std::fabs(eatt.getQ3() + 0.258819) > eps)
    return false;
  eraxis = scanner->getScannerHead(0)->cfg_device_rotateAxis;
  if (eraxis[0] != 1 || eraxis[1] != 0 || eraxis[2] != 0)
    return false;
  eraxis = scanner->getScannerHead(1)->cfg_device_rotateAxis;
  if (eraxis[0] != 0 || eraxis[1] != 1 || eraxis[2] != 0)
    return false;
  eraxis = scanner->getScannerHead(2)->cfg_device_rotateAxis;
  if (eraxis[0] != 0 || eraxis[1] != 1 || eraxis[2] != 0)
    return false;
  // Load and validate unexistent scanner
  try {
    scanner = std::dynamic_pointer_cast<Scanner>(
      loader.getAssetById("scanner", "_unexistent_scanner_", nullptr));
    return false; // Must not be reached, exception before
  } catch (HeliosException& hex) {
  }
  return true;
}

bool
AssetLoadingTest::testPlatformLoading()
{
  // Prepare platform loading
  std::string testPlatformsPath = "data/test/test_platforms.xml";
  std::vector<std::string> assetsPath = { "assets/" };
  XmlAssetsLoader loader(testPlatformsPath, assetsPath);
  // Load and validate Quadrocopter UAV
  std::shared_ptr<Platform> platf = std::dynamic_pointer_cast<Platform>(
    loader.getAssetById("platform", "quadcopter", nullptr));
  std::shared_ptr<HelicopterPlatform> hplatf =
    std::dynamic_pointer_cast<HelicopterPlatform>(platf);
  if (hplatf == nullptr)
    return false;
  if (platf->name != "Quadrocopter UAV")
    return false;
  if (hplatf->mCfg_drag != 0.0099)
    return false;
  if (hplatf->ef_xy_max != 0.099)
    return false;
  if (hplatf->cfg_speedup_magnitude != 1.99)
    return false;
  if (hplatf->cfg_slowdown_magnitude != 1.99)
    return false;
  if (hplatf->cfg_slowdown_dist_xy != 4.99)
    return false;
  if (std::fabs(hplatf->cfg_pitch_base + 0.0959931) > eps)
    return false;
  if (std::fabs(hplatf->cfg_roll_speed - 0.5087635) > eps)
    return false;
  if (std::fabs(hplatf->cfg_pitch_speed - 1.5086626) > eps)
    return false;
  if (std::fabs(hplatf->cfg_yaw_speed - 1.4999360) > eps)
    return false;
  if (std::fabs(hplatf->cfg_max_roll_offset - 0.4433136) > eps)
    return false;
  if (std::fabs(hplatf->cfg_max_pitch_offset - 0.6038839) > eps)
    return false;
  glm::dvec3 mpdiff =
    platf->cfg_device_relativeMountPosition - glm::dvec3(0, 0, 0.21);
  if (std::fabs(mpdiff[0]) > eps || std::fabs(mpdiff[1]) > eps ||
      std::fabs(mpdiff[2]) > eps)
    return false;
  Rotation matt = platf->cfg_device_relativeMountAttitude;
  if (std::fabs(matt.getQ0() - 0) > eps || std::fabs(matt.getQ1() - 0) > eps ||
      std::fabs(matt.getQ2() + 1) > eps || std::fabs(matt.getQ3() - 0) > eps)
    return false;
  // Load and validate Cirrus SR-22
  platf = std::dynamic_pointer_cast<Platform>(
    loader.getAssetById("platform", "sr22", nullptr));
  if (platf->name != "Cirrus SR-22")
    return false;
  mpdiff = platf->cfg_device_relativeMountPosition - glm::dvec3(0, 0, 0.7);
  if (std::fabs(mpdiff[0]) > eps || std::fabs(mpdiff[1]) > eps ||
      std::fabs(mpdiff[2]) > eps)
    return false;
  matt = platf->cfg_device_relativeMountAttitude;
  if (std::fabs(matt.getQ0() - 0.5) > eps ||
      std::fabs(matt.getQ1() - 0.5) > eps ||
      std::fabs(matt.getQ2() - 0.5) > eps ||
      std::fabs(matt.getQ3() + 0.5) > eps)
    return false;
  if (platf->positionXNoiseSource->getClipMin() != 0.0)
    return false;
  if (platf->positionXNoiseSource->getClipMax() != 0.0)
    return false;
  if (platf->positionXNoiseSource->isClipEnabled())
    return false;
  if (platf->positionXNoiseSource->getFixedLifespan() != 5)
    return false;
  std::shared_ptr<NormalNoiseSource<double>> nns =
    std::dynamic_pointer_cast<NormalNoiseSource<double>>(
      platf->positionXNoiseSource);
  if (nns == nullptr)
    return false;
  if (std::dynamic_pointer_cast<UniformNoiseSource<double>>(
        platf->positionXNoiseSource) != nullptr)
    return false;
  if (nns->getMean() != 0.01)
    return false;
  if (nns->getStdev() != 0.021)
    return false;
  if (platf->positionYNoiseSource->getClipMin() != -0.01)
    return false;
  if (platf->positionYNoiseSource->getClipMax() != 0.0)
    return false;
  if (platf->positionYNoiseSource->isClipEnabled())
    return false;
  if (platf->positionYNoiseSource->getFixedLifespan() != 7)
    return false;
  nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(
    platf->positionYNoiseSource);
  if (nns == nullptr)
    return false;
  if (std::dynamic_pointer_cast<UniformNoiseSource<double>>(
        platf->positionYNoiseSource) != nullptr)
    return false;
  if (nns->getMean() != -0.01)
    return false;
  if (nns->getStdev() != 0.019)
    return false;
  if (platf->positionZNoiseSource->getClipMin() != -0.03)
    return false;
  if (platf->positionZNoiseSource->getClipMax() != 0.03)
    return false;
  if (!platf->positionZNoiseSource->isClipEnabled())
    return false;
  if (platf->positionZNoiseSource->getFixedLifespan() != 1)
    return false;
  nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(
    platf->positionZNoiseSource);
  if (nns == nullptr)
    return false;
  if (std::dynamic_pointer_cast<UniformNoiseSource<double>>(
        platf->positionZNoiseSource) != nullptr)
    return false;
  if (nns->getMean() != 0.0)
    return false;
  if (nns->getStdev() != 0.02)
    return false;
  if (platf->attitudeXNoiseSource->getClipMin() != 0.0)
    return false;
  if (platf->attitudeXNoiseSource->getClipMax() != 0.0)
    return false;
  if (platf->attitudeXNoiseSource->isClipEnabled())
    return false;
  if (platf->attitudeXNoiseSource->getFixedLifespan() != 1)
    return false;
  nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(
    platf->attitudeXNoiseSource);
  if (nns == nullptr)
    return false;
  if (std::dynamic_pointer_cast<UniformNoiseSource<double>>(
        platf->attitudeXNoiseSource) != nullptr)
    return false;
  if (nns->getMean() != 0.0)
    return false;
  if (nns->getStdev() != 0.001)
    return false;
  if (platf->attitudeYNoiseSource->getClipMin() != 0.0)
    return false;
  if (platf->attitudeYNoiseSource->getClipMax() != 0.0)
    return false;
  if (platf->attitudeYNoiseSource->isClipEnabled())
    return false;
  if (platf->attitudeYNoiseSource->getFixedLifespan() != 3)
    return false;
  nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(
    platf->attitudeYNoiseSource);
  if (nns == nullptr)
    return false;
  if (std::dynamic_pointer_cast<UniformNoiseSource<double>>(
        platf->attitudeYNoiseSource) != nullptr)
    return false;
  if (nns->getMean() != 0.0)
    return false;
  if (nns->getStdev() != 0.001)
    return false;
  if (platf->attitudeZNoiseSource->getClipMin() != 0.0)
    return false;
  if (platf->attitudeZNoiseSource->getClipMax() != 0.0)
    return false;
  if (platf->attitudeZNoiseSource->isClipEnabled())
    return false;
  if (platf->attitudeZNoiseSource->getFixedLifespan() != 11)
    return false;
  nns = std::dynamic_pointer_cast<NormalNoiseSource<double>>(
    platf->attitudeZNoiseSource);
  if (nns == nullptr)
    return false;
  if (std::dynamic_pointer_cast<UniformNoiseSource<double>>(
        platf->attitudeZNoiseSource) != nullptr)
    return false;
  if (nns->getMean() != 0.0)
    return false;
  if (nns->getStdev() != 0.001)
    return false;
  // Load and validate Tractor
  platf = std::dynamic_pointer_cast<Platform>(
    loader.getAssetById("platform", "tractor"));
  if (platf->name != "Tractor")
    return false;
  mpdiff = platf->cfg_device_relativeMountPosition - glm::dvec3(0, 1, 4);
  if (std::fabs(mpdiff[0]) > eps || std::fabs(mpdiff[1]) > eps ||
      std::fabs(mpdiff[2]) > eps)
    return false;
  matt = platf->cfg_device_relativeMountAttitude;
  if (std::fabs(matt.getQ0() - 0.6830127) > eps ||
      std::fabs(matt.getQ1() + 0.1830127) > eps ||
      std::fabs(matt.getQ2() - 0.1830127) > eps ||
      std::fabs(matt.getQ3() + 0.6830127) > eps)
    return false;
  std::shared_ptr<GroundVehiclePlatform> gplatf =
    std::dynamic_pointer_cast<GroundVehiclePlatform>(platf);
  if (gplatf == nullptr)
    return false;
  if (gplatf->mCfg_drag != 0.00499)
    return false;
  // Load and validate Tripod
  platf = std::dynamic_pointer_cast<Platform>(
    loader.getAssetById("platform", "tripod", nullptr));
  if (platf->name != "TLS Tripod")
    return false;
  mpdiff = platf->cfg_device_relativeMountPosition - glm::dvec3(0, 0, 1.5);
  if (std::fabs(mpdiff[0]) > eps || std::fabs(mpdiff[1]) > eps ||
      std::fabs(mpdiff[2]) > eps)
    return false;
  return true;
}

bool
AssetLoadingTest::testObjLoading()
{
  // TODO Rethink : Implement
  return true;
}

bool
AssetLoadingTest::testVoxelLoading()
{
  // TODO Rethink : Implement
  return true;
}

bool
AssetLoadingTest::testDetailedVoxelLoading()
{
  // TODO Rethink : Implement
  return true;
}

bool
AssetLoadingTest::testTiffLoading()
{
  // TODO Rethink : Implement
  return true;
}

bool
AssetLoadingTest::testSceneLoading()
{
  // TODO Rethink : Implement
  return true;
}

bool
AssetLoadingTest::testSurveyLoading()
{
  // TODO Rethink : Implement
  return true;
}

}
