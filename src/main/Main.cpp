#include <ArgumentsParser.h>
#include <helios_version.h>
#include <main/LidarSim.h>
#include <main/MainHelp.h>
#include <noise/RandomnessGenerator.h>

#ifdef PCL_BINDING
#include <demo/DemoSelector.h>
#endif

#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_GlobalVars.h>
#include <dataanalytics/HDA_GlobalVarsReporter.h>
#endif

#include <boost/filesystem.hpp>
#include <gdal_priv.h>
#include <logger_core.hpp>

#include <iomanip>

namespace fs = boost::filesystem;

// ***  DECLARATIONS  *** //
// ********************** //

// ***  MAIN : EXECUTION ENTRY POINT  *** //
// ************************************** //
/**
 * @brief Main method. It is, the entry point for execution
 * @param argc How many arguments
 * @param argv Arguments as array of strings
 * @return Exit status 0 if execution was successfully completed, distinct
 *  than 0 otherwise
 */
int
main(int argc, char** argv)
{
  // Build arguments parser
  ArgumentsParser ap(argc, argv);

  // Handle different execution branches
  bool done = false;
  if (argc == 1 || ap.parseHelpRequest()) { // Help execution branch
    helios::main::printMainHelp();
    done = true;
  }
  if (ap.parseVersionRequest()) { // Print version and finish
    std::cout << getHeliosFullVersion() << std::endl;
    return EXIT_SUCCESS;
  } else if (argc > 1) {
    std::string demo = ap.parseDemoRequest();
    std::string demoSurveyPath = ap.parseDemoSurveyPath();
    std::string demoAssetsPath = ap.parseDemoAssetsPath();
    // Test execution branch
    if (ap.parseTestRequest()) {
      std::cout << "Running HELIOS++ tests via --test is deprecated. Run the "
                   "test executable helios_test instead."
                << std::endl;
      done = true;
      return EXIT_FAILURE;
    }
#ifdef PCL_BINDING
    // Demo execution branch
    else if (demo != "NULL") {
      // Load drivers
      GDALAllRegister(); // Load All known GDAL Drivers

      // Run demo
      HeliosDemos::DemoSelector::getInstance()->select(
        demo, demoSurveyPath, demoAssetsPath);
      done = true;
    }
#endif
  }

  // Main simulation execution branch
  if (!done) {
    // Load drivers
    GDALAllRegister(); // Load All known GDAL Drivers

    // Deprecated CLI: logging is silent / no-op.
    logging::configure_silent();
    // Handle default randomness generator
    std::string seed = ap.parseSeed();
    if (seed != "")
      setDefaultRandomnessGeneratorSeed(seed);

    // Load lidar simulation
    helios::main::LidarSim app;
    app.init(ap.parseSurveyPath(),
             ap.parseAssetsPath(),
             ap.parseOutputPath(),
             ap.parseWriteWaveform(),
             ap.parseWritePulse(),
             ap.parseCalcEchowidth(),
             ap.parseParallelizationStrategy(),
             ap.parseNJobs(),
             ap.parseChunkSize(),
             ap.parseWarehouseFactor(),
             ap.parseFullWaveNoise(),
             ap.parseSplitByChannel(),
             ap.parseDisablePlatformNoise(),
             ap.parseDisableLegNoise(),
             ap.parseLasOutput(),
             ap.parseLas10(),
             ap.parseZipOutput(),
             ap.parseFixedIncidenceAngle(),
             ap.parseGpsStartTime(),
             ap.parseLasScale(),
             ap.parseKDTreeType(),
             ap.parseKDTreeJobs(),
             ap.parseKDTreeGeometricJobs(),
             ap.parseSAHLossNodes(),
             ap.parseLegacyEnergyModel());
  }

#if DATA_ANALYTICS >= 2
  helios::analytics::HDA_GlobalVarsReporter reporter(helios::analytics::HDA_GV);
  reporter.print();
#endif

  // Return successful exit status code (0)
  return EXIT_SUCCESS;
}
