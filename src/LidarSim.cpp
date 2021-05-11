#include "LidarSim.h"

#include <iostream>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "logging.hpp"

#include "Survey.h"
#include "SurveyPlayback.h"
#include "XmlSurveyLoader.h"
#include "typedef.h"
#include <ArgumentsParser.h>
#include <gdal_priv.h>
#include <noise/RandomnessGenerator.h>
#include <TimeWatcher.h>
#include <helios_version.h>
#include <AbstractDetector.h>
#include <FileUtils.h>
#include <armadillo>
#include <iomanip>

using namespace std;
void doTests(std::string const & testDir);

// LOGGING FLAGS (DO NOT MODIFY HERE BUT IN logging.hpp makeDefault())
bool    logging::LOGGING_SHOW_TRACE, logging::LOGGING_SHOW_DEBUG,
        logging::LOGGING_SHOW_INFO, logging::LOGGING_SHOW_WARN,
        logging::LOGGING_SHOW_ERR;

void printHelp(){
    cout << "helios++ help:\n\n"
        << "\tSyntax: helios++ <survey_file_path> [OPTIONAL ARGUMENTS]\n\n"
        << "\tOPTIONAL ARGUMENTS:\n\n"
        << "\t\t-h or --help : Show this help\n\n"
        << "\t\t--test : Run tests to check helios++ behaves as expected\n\n"
        << "\t\t--unzip <input_path> <output_path>\n"
        << "\t\t\tDecompress the file at input path and write it decompressed "
        << "at output path.\n"
        << "\t\t\tFile at input path must be the compressed output of helios++"
        << "\n\n"
        << "\t\t--assets <dir_path> : Specify the path to assets directory\n"
        << "\t\t\tBy default: ./assets/\n\n"
        << "\t\t--output <dir_path> : Specify the path to output directory\n"
        << "\t\t\tBy default: ./output/\n\n"
        << "\t\t--writeWaveform : Use this flag to enable full waveform "
        << "writing\n"
        << "\t\t\tBy default waveform is NOT written to output file\n\n"
        << "\t\t--calcEchowidth : Use this flag to enable full waveform "
        << "fitting\n"
        << "\t\t\tBy default the full waveform is NOT fitted\n\n"
        << "\t\t--fullwaveNoise : Use this flag to add noise when computing "
        << "full waveform\n"
        << "\t\t\tBy default: full waveform noise is disabled\n\n"
        << "\t\t--fixedIncidenceAngle : Use this flag to use fixed incidence "
        << "angle\n"
        << "\t\t\tFixed incidence angle of exactly 0.0 will be considered for "
        << "all intersections\n\n"
        << "\t\t--seed <seed>: Specify the seed for randomness generation\n"
        << "\t\t\tIt can be an intenger, a decimal or a timestamp with format "
        << "\n\t\t\tYYYY-mm-DD HH::MM::SS\n"
        << "\t\t\t\tBy default: a random seed is generated\n\n"
        << "\t\t--lasOutput : Use this flag to generate the output point cloud "
		   "in LAS format (v 1.4)\n\n"
        << "\t\t--las10: Use this flag to write in LAS format (v 1.0)\n\n"
        << "\t\t--zipOutput : Use this flag to generate compressed output\n\n"
        << "\t\t--lasScale : Specify the decimal scale factor for LAS output"
        << "\n\n"
        << "\t\t-j or --njobs or --nthreads <integer> : Specify the number of"
        << "\n\t\t\tjobs to be used to compute the simulation\n"
        << "\t\t\t\tBy default: all available threads are used\n\n"
        << "\t\t--rebuildScene : Force scene rebuild even when a previously\n"
        << "\t\t\tbuilt scene is available\n"
        << "\t\t\t\tBy default: previous scene is used if found\n\n"
        << "\t\t--disablePlatformNoise : Disable platform noise, no matter\n"
        << "\t\t\twhat is specified on XML files\n"
        << "\t\t\t\tBy default: XML specifications are considered\n\n"
        << "\t\t--disableLegNoise : Disable leg noise, no matter what is\n"
        << "\t\t\tspecified on XML files\n"
        << "\t\t\t\tBy default: XML specifications are considered\n\n"
        << "\t\t--logFile : Logging will be outputted to a file, not only\n"
        << "\t\t\tto standard output.\n"
        << "\t\t\t\tBy default: logging will be written to standard output\n\n"
        << "\t\t--logFileOnly : Logging will be outputted ONLY to a file\n"
        << "\t\t\tBy default: logging will be outputted to standard output\n\n"
        << "\t\t--silent : Disable logging output\n"
        << "\t\t\tBy default: only information and errors are reported\n\n"
        << "\t\t-q or --quiet : Specify the verbosity level to errors only\n"
        << "\t\t\tBy default: only information and errors are reported\n\n"
        << "\t\t-v : Specify the verbosity level to errors, information and "
        << "warnings\n"
        << "\t\t\tBy default: only information and errors are reported\n\n"
        << "\t\t-v2 or -vv : Specify the verbosity level to report all "
        <<  "messages\n"
        << "\t\t\tBy default: only information and errors are reported\n\n"
        << endl;
}

int main(int argc, char** argv) {
	ArgumentsParser ap(argc, argv);
	bool done = false;
	if(argc == 1 || ap.parseHelpRequest()){
	    printHelp();
	    done = true;
	}
	else if(argc > 1){
	    std::string inputPath, outputPath;
	    if(ap.parseTestRequest()) {
            doTests(ap.parseTestDir());
            done = true;
        }
	    else if(ap.parseUnzip(&inputPath, &outputPath)){
	        FileUtils::unzipFile(inputPath, outputPath);
	        done = true;
	    }
	}
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
        string seed = ap.parseSeed();
        if(seed != ""){
            stringstream ss;
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
            ap.parseCalcEchowidth(),
            ap.parseNJobs(),
            ap.parseFullWaveNoise(),
            ap.parseDisablePlatformNoise(),
            ap.parseDisableLegNoise(),
            ap.parseRebuildScene(),
            ap.parseLasOutput(),
            ap.parseLas10(),
            ap.parseZipOutput(),
            ap.parseFixedIncidenceAngle(),
            ap.parseLasScale()
        );
    }

	return EXIT_SUCCESS;
}

void LidarSim::init(
    string surveyPath,
    string assetsPath,
    string outputPath,
    bool writeWaveform,
    bool calcEchowidth,
    size_t njobs,
    bool fullWaveNoise,
    bool platformNoiseDisabled,
    bool legNoiseDisabled,
    bool rebuildScene,
    bool lasOutput,
    bool las10,
    bool zipOutput,
    bool fixedIncidenceAngle,
    double lasScale
){
	std::stringstream ss;
	ss  << "surveyPath: \"" << surveyPath << "\"\n"
	    << "assetsPath: \"" << assetsPath << "\"\n"
	    << "outputPath: \"" << outputPath << "\"\n"
	    << "writeWaveform: " << writeWaveform << "\n"
            << "calcEchowidth: " << calcEchowidth << "\n"
	    << "fullWaveNoise: " << fullWaveNoise << "\n"
	    << "njobs: " << njobs << "\n"
	    << "platformNoiseDisabled: " << platformNoiseDisabled << "\n"
	    << "legNoiseDisabled: " << legNoiseDisabled << "\n"
	    << "rebuildScene: " << rebuildScene << "\n"
	    << "lasOutput: " << lasOutput << "\n"
            << "las10: " << las10 << "\n"
	    << "fixedIncidenceAngle: " << fixedIncidenceAngle << std::endl;
    logging::INFO(ss.str());

	// Load survey description from XML file:
 	shared_ptr<XmlSurveyLoader> xmlreader(
 	    new XmlSurveyLoader(surveyPath, assetsPath)
 	);
	shared_ptr<Survey> survey = xmlreader->load(
	    legNoiseDisabled,
	    rebuildScene
    );
	survey->scanner->setWriteWaveform(writeWaveform);
	survey->scanner->setCalcEchowidth(calcEchowidth);
	survey->scanner->setFullWaveNoise(fullWaveNoise);
	survey->scanner->setPlatformNoiseDisabled(platformNoiseDisabled);
	survey->scanner->setFixedIncidenceAngle(fixedIncidenceAngle);
	survey->scanner->detector->lasOutput = lasOutput;
        survey->scanner->detector->las10 = las10;
	survey->scanner->detector->zipOutput = zipOutput;
	survey->scanner->detector->lasScale = lasScale;
	if (survey == nullptr) {
		logging::ERR("Failed to load survey!");
		exit(-1);
	}

	shared_ptr<SurveyPlayback> playback = std::make_shared<SurveyPlayback>(
        survey,
        outputPath,
        njobs,
        lasOutput,
        zipOutput
	);

    logging::INFO("Running simulation...");

	TimeWatcher tw;
	tw.start();
	playback->start();
	tw.stop();
	tw.reportFormat("Total simulation time: ");
}