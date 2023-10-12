#include <main/LidarSim.h>
#include <main/MainHelp.h>
#include <ArgumentsParser.h>
#include <noise/RandomnessGenerator.h>
#include <helios_version.h>
#include <filems/util/FileUtils.h>

#ifdef PCL_BINDING
#include <demo/DemoSelector.h>
#endif

#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_GlobalVars.h>
#include <dataanalytics/HDA_GlobalVarsReporter.h>
#endif

#include <gdal_priv.h>
#include <boost/filesystem.hpp>

#include <iomanip>

using helios::main::LidarSim;

namespace fs = boost::filesystem;




// ***  DECLARATIONS  *** //
// ********************** //
/**
 * @brief Run HELIOS++ tests
 * @param testDir Path to the directory containing test data
 */
void doTests(std::string const & testDir);

// LOGGING FLAGS (DO NOT MODIFY HERE BUT IN logging.hpp makeDefault())
bool    logging::LOGGING_SHOW_TRACE,    logging::LOGGING_SHOW_DEBUG,
        logging::LOGGING_SHOW_INFO,     logging::LOGGING_SHOW_TIME,
        logging::LOGGING_SHOW_WARN,     logging::LOGGING_SHOW_ERR;




// ***  MAIN : EXECUTION ENTRY POINT  *** //
// ************************************** //
/**
 * @brief Main method. It is, the entry point for execution
 * @param argc How many arguments
 * @param argv Arguments as array of strings
 * @return Exit status 0 if execution was successfully completed, distinct
 *  than 0 otherwise
 */
int main(int argc, char** argv) {
    // Build arguments parser
    ArgumentsParser ap(argc, argv);

    // Handle different execution branches
    bool done = false;
    if(argc == 1 || ap.parseHelpRequest()){ // Help execution branch
        helios::main::printMainHelp();
        done = true;
    }
    if(ap.parseVersionRequest()){  // Print version and finish
        std::cout << getHeliosFullVersion() << std::endl;
        return EXIT_SUCCESS;
    }
    else if(argc > 1){
        std::string inputPath, outputPath;
        std::string demo = ap.parseDemoRequest();
        std::string demoSurveyPath = ap.parseDemoSurveyPath();
        std::string demoAssetsPath = ap.parseDemoAssetsPath();
        // Test execution branch
        if(ap.parseTestRequest()) {
            doTests(ap.parseTestDir());
            done = true;
        }
#ifdef PCL_BINDING
        // Demo execution branch
        else if(demo != "NULL"){
            // Load drivers
            GDALAllRegister();  // Load All known GDAL Drivers

            // Run demo
            HeliosDemos::DemoSelector::getInstance()->select(
                demo,
                demoSurveyPath,
                demoAssetsPath
            );
            done = true;
        }
#endif
        // Unzip execution branch
        else if(ap.parseUnzip(&inputPath, &outputPath)){
            FileUtils::unzipFile(inputPath, outputPath);
            done = true;
        }
    }

    // Main simulation execution branch
    if(!done){
        // Load drivers
        GDALAllRegister();  // Load All known GDAL Drivers

        // Configure logging
        ap.parseLoggingVerbosity();
        time_t t = std::time(nullptr);
        std::tm *tm = std::localtime(&t);
        std::stringstream ss;
        ss << "helios_" << std::put_time(tm, "%Y-%m-%d_%H-%M-%S") << ".log";
        logging::configure({
            { "type", ap.parseLoggingOutputMode() },
            { "file_name", ss.str() },
            { "reopen_interval", "1" }
        });
        ss.str("");

        // Show version
        ss << "HELIOS++ VERSION "<< getHeliosVersion() << "\n";
        logging::INFO(ss.str());

        // Show current working directory
        ss.str("");
        ss << "CWD: " << boost::filesystem::current_path();
        logging::INFO(ss.str());

        // Handle default randomness generator
        std::string seed = ap.parseSeed();
        if(seed != ""){
            std::stringstream ss;
            ss << "seed: " << seed;
            logging::INFO(ss.str());
            setDefaultRandomnessGeneratorSeed(seed);
        }
        else{
            logging::INFO("seed: AUTO");
        }

        // Load lidar simulation
        LidarSim app;
        app.init(
            ap.parseSurveyPath(),
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
            ap.parseRebuildScene(),
            ap.parseLasOutput(),
            ap.parseLas10(),
            ap.parseZipOutput(),
            ap.parseFixedIncidenceAngle(),
            ap.parseGpsStartTime(),
            ap.parseLasScale(),
            ap.parseKDTreeType(),
            ap.parseKDTreeJobs(),
            ap.parseKDTreeGeometricJobs(),
            ap.parseSAHLossNodes()
        );
    }

#if DATA_ANALYTICS >= 2
    helios::analytics::HDA_GlobalVarsReporter reporter(
        helios::analytics::HDA_GV
    );
    reporter.print();
#endif

    // Return successful exit status code (0)
    return EXIT_SUCCESS;
}
