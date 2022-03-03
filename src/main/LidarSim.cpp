#include <main/LidarSim.h>

#include <iostream>

#include "logging.hpp"

#include "Survey.h"
#include "SurveyPlayback.h"
#include "XmlSurveyLoader.h"
#include <TimeWatcher.h>
#include <scanner/detector/PulseThreadPoolFactory.h>
#include <filems/facade/FMSFacade.h>
#include <filems/write/MeasurementWriter.h>
#include <filems/write/TrajectoryWriter.h>


namespace fms = helios::filems;

namespace helios { namespace main{


using std::make_shared;



void LidarSim::init(
    std::string surveyPath,
    std::string assetsPath,
    std::string outputPath,
    bool writeWaveform,
    bool calcEchowidth,
    int parallelizationStrategy,
    size_t njobs,
    int chunkSize,
    int warehouseFactor,
    bool fullWaveNoise,
    bool platformNoiseDisabled,
    bool legNoiseDisabled,
    bool rebuildScene,
    bool lasOutput,
    bool las10,
    bool zipOutput,
    bool fixedIncidenceAngle,
    std::string gpsStartTime,
    double lasScale,
    int kdtType,
    size_t kdtJobs,
    size_t kdtGeomJobs,
    size_t sahLossNodes
){
    // Info about execution arguments
    std::stringstream ss;
	ss  << "surveyPath: \"" << surveyPath << "\"\n"
	    << "assetsPath: \"" << assetsPath << "\"\n"
	    << "outputPath: \"" << outputPath << "\"\n"
	    << "writeWaveform: " << writeWaveform << "\n"
        << "calcEchowidth: " << calcEchowidth << "\n"
	    << "fullWaveNoise: " << fullWaveNoise << "\n"
	    << "parallelization: " << parallelizationStrategy << "\n"
	    << "njobs: " << njobs << "\n"
	    << "chunkSize: " << chunkSize << "\n"
	    << "warehouseFactor: " << warehouseFactor << "\n"
	    << "platformNoiseDisabled: " << platformNoiseDisabled << "\n"
	    << "legNoiseDisabled: " << legNoiseDisabled << "\n"
	    << "rebuildScene: " << rebuildScene << "\n"
	    << "lasOutput: " << lasOutput << "\n"
        << "las10: " << las10 << "\n"
	    << "fixedIncidenceAngle: " << fixedIncidenceAngle << "\n"
	    << "gpsStartTime: " << gpsStartTime << "\n"
	    << "kdtType: " << kdtType << "\n"
	    << "kdtJobs: " << kdtJobs << "\n"
	    << "kdtGeomJobs: " << kdtGeomJobs << "\n"
	    << "sahLossNodes: " << sahLossNodes
	    << std::endl;
    logging::INFO(ss.str());

	// Load survey description from XML file:
 	std::shared_ptr<XmlSurveyLoader> xmlreader(
 	    new XmlSurveyLoader(surveyPath, assetsPath)
 	);
 	xmlreader->sceneLoader.kdtFactoryType = kdtType;
 	xmlreader->sceneLoader.kdtNumJobs = kdtJobs;
 	xmlreader->sceneLoader.kdtGeomJobs = kdtGeomJobs;
    xmlreader->sceneLoader.kdtSAHLossNodes = sahLossNodes;
	std::shared_ptr<Survey> survey = xmlreader->load(
	    legNoiseDisabled,
	    rebuildScene
    );
    if (survey == nullptr) {
        logging::ERR("Failed to load survey!");
        exit(-1);
	}

    // Configure scanner from input arguments
	survey->scanner->setWriteWaveform(writeWaveform);
	survey->scanner->setCalcEchowidth(calcEchowidth);
	survey->scanner->setFullWaveNoise(fullWaveNoise);
	survey->scanner->setPlatformNoiseDisabled(platformNoiseDisabled);
	survey->scanner->setFixedIncidenceAngle(fixedIncidenceAngle);
	// TODO Rethink : Implement main package with building methods for FMS ...
	// ... and other components
	shared_ptr<fms::FMSFacade> fms = make_shared<fms::FMSFacade>();
	survey->scanner->fms = fms;
	survey->scanner->detector->fms = fms;
	fms::FMSWriteFacade &fmsWrite = fms->write;
	fmsWrite.setMeasurementWriter(make_shared<fms::MeasurementWriter>());
	fmsWrite.getMeasurementWriter()->setScanner(survey->scanner);
	fmsWrite.setMeasurementWriterLasOutput(lasOutput);
    fmsWrite.setMeasurementWriterLas10(las10);
    fmsWrite.setMeasurementWriterZipOutput(zipOutput);
    fmsWrite.setMeasurementWriterLasScale(lasScale);
    fmsWrite.setTrajectoryWriter(make_shared<fms::TrajectoryWriter>());
    fmsWrite.setFullWaveformWriter(make_shared<fms::FullWaveformWriter>());
    // TODO Rethink : Use outputPath variable to define FMS root directory

	// Build thread pool for parallel computation
	/*
     * Number of threads available in the system.
     * May return 0 when not able to detect
     */
    unsigned numSysThreads = std::thread::hardware_concurrency();
    size_t const poolSize = (njobs == 0) ? numSysThreads-1 : njobs-1;
    PulseThreadPoolFactory ptpf(
        parallelizationStrategy,
        poolSize,
        survey->scanner->detector->cfg_device_accuracy_m,
        chunkSize,
        warehouseFactor
    );
    std::shared_ptr<PulseThreadPoolInterface> pulseThreadPool =
        ptpf.makePulseThreadPool();

	std::shared_ptr<SurveyPlayback> playback=std::make_shared<SurveyPlayback>(
        survey,
        fms,
        parallelizationStrategy,
        pulseThreadPool,
        std::abs(chunkSize),
        gpsStartTime,
        lasOutput,
        las10,
        zipOutput
	);

    logging::INFO("Running simulation...");
	TimeWatcher tw;
	tw.start();
	playback->start();
	tw.stop();
	tw.reportFormat("Total simulation time: ");
}

}}
