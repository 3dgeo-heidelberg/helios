#include <main/LidarSim.h>

#include <iostream>

#include "logging.hpp"

#include "Survey.h"
#include "XmlSurveyLoader.h"
#include <TimeWatcher.h>
#include <filems/facade/FMSFacade.h>
#include <filems/factory/FMSFacadeFactory.h>
#include <scanner/detector/PulseThreadPoolFactory.h>

namespace fms = helios::filems;

namespace helios {
namespace main {

void
LidarSim::init(std::string surveyPath,
               std::vector<std::string> assetsPath,
               std::string outputPath,
               bool writeWaveform,
               bool writePulse,
               bool calcEchowidth,
               int parallelizationStrategy,
               size_t njobs,
               int chunkSize,
               int warehouseFactor,
               bool fullWaveNoise,
               bool splitByChannel,
               bool platformNoiseDisabled,
               bool legNoiseDisabled,
               bool rebuildScene,
               bool writeScene,
               bool lasOutput,
               bool las10,
               bool zipOutput,
               bool fixedIncidenceAngle,
               std::string gpsStartTime,
               double lasScale,
               int kdtType,
               size_t kdtJobs,
               size_t kdtGeomJobs,
               size_t sahLossNodes,
               bool const legacyEnergyModel)
{
  // Info about execution arguments
  std::stringstream ss;
  ss << "surveyPath: \"" << surveyPath << "\"\n"
     << "assetsPath: [";
  for (const auto path : assetsPath) {
    ss << "\"" << path << "\", ";
  }
  ss << "]\n"
     << "outputPath: \"" << outputPath << "\"\n"
     << "writeWaveform: " << writeWaveform << "\n"
     << "writePulse: " << writePulse << "\n"
     << "calcEchowidth: " << calcEchowidth << "\n"
     << "fullWaveNoise: " << fullWaveNoise << "\n"
     << "splitByChannel: " << splitByChannel << "\n"
     << "parallelization: " << parallelizationStrategy << "\n"
     << "njobs: " << njobs << "\n"
     << "chunkSize: " << chunkSize << "\n"
     << "warehouseFactor: " << warehouseFactor << "\n"
     << "platformNoiseDisabled: " << platformNoiseDisabled << "\n"
     << "legNoiseDisabled: " << legNoiseDisabled << "\n"
     << "rebuildScene: " << rebuildScene << "\n"
     << "writeScene: " << writeScene << "\n"
     << "lasOutput: " << lasOutput << "\n"
     << "las10: " << las10 << "\n"
     << "fixedIncidenceAngle: " << fixedIncidenceAngle << "\n"
     << "gpsStartTime: " << gpsStartTime << "\n"
     << "kdtType: " << kdtType << "\n"
     << "kdtJobs: " << kdtJobs << "\n"
     << "kdtGeomJobs: " << kdtGeomJobs << "\n"
     << "sahLossNodes: " << sahLossNodes << std::endl;
  logging::INFO(ss.str());

  // Load survey description from XML file:
  std::shared_ptr<XmlSurveyLoader> xmlreader =
    std::make_shared<XmlSurveyLoader>(surveyPath, assetsPath, writeScene);
  xmlreader->sceneLoader.kdtFactoryType = kdtType;
  xmlreader->sceneLoader.kdtNumJobs = kdtJobs;
  xmlreader->sceneLoader.kdtGeomJobs = kdtGeomJobs;
  xmlreader->sceneLoader.kdtSAHLossNodes = sahLossNodes;
  std::shared_ptr<Survey> survey =
    xmlreader->load(legNoiseDisabled, rebuildScene);
  if (survey == nullptr) {
    logging::ERR("Failed to load survey!");
    exit(-1);
  }

  // Configure scanner from input arguments
  survey->scanner->setWriteWaveform(writeWaveform);
  survey->scanner->setWritePulse(writePulse);
  survey->scanner->setCalcEchowidth(calcEchowidth);
  survey->scanner->setFullWaveNoise(fullWaveNoise);
  survey->scanner->setPlatformNoiseDisabled(platformNoiseDisabled);
  survey->scanner->setFixedIncidenceAngle(fixedIncidenceAngle);

  // Build main facade for File Management System, associated to the survey
  std::shared_ptr<fms::FMSFacade> fms = fms::FMSFacadeFactory().buildFacade(
    outputPath, lasScale, lasOutput, las10, zipOutput, splitByChannel, *survey);

  // Build thread pool for parallel computation
  /*
   * Number of threads available in the system.
   * May return 0 when not able to detect
   */
  unsigned numSysThreads = std::thread::hardware_concurrency();
  std::size_t const poolSize = (njobs == 0) ? numSysThreads - 1 : njobs - 1;
  PulseThreadPoolFactory ptpf(
    parallelizationStrategy,
    poolSize,
    survey->scanner->getDetector()->cfg_device_accuracy_m,
    chunkSize,
    warehouseFactor);
  std::shared_ptr<PulseThreadPoolInterface> pulseThreadPool =
    ptpf.makePulseThreadPool();

  // Build the survey playback simulation itself
  std::shared_ptr<SurveyPlayback> playback =
    std::make_shared<SurveyPlayback>(survey,
                                     fms,
                                     parallelizationStrategy,
                                     pulseThreadPool,
                                     std::abs(chunkSize),
                                     gpsStartTime,
                                     legacyEnergyModel);

  // Start simulation
  logging::INFO("Running simulation...");
  TimeWatcher tw;
  tw.start();
  playback->start();
  tw.stop();
  tw.reportFormat("Total simulation time: ");

  // Disconnect FMS
  fms->disconnect();

  // Release resources before finishing
  release(playback);
}

void
LidarSim::release(std::shared_ptr<SurveyPlayback> sp)
{
  // Release scanner
  std::shared_ptr<Scanner> sc = sp->mSurvey->scanner;
  sc->randGen1 = nullptr;
  sc->randGen2 = nullptr;
  sc->intersectionHandlingNoiseSource = nullptr;
  sc->setAllDetectors(nullptr);
  sc->spp = nullptr;
  sc->allOutputPaths = nullptr;
  sc->allMeasurements = nullptr;
  sc->allTrajectories = nullptr;
  sc->allMeasurementsMutex = nullptr;
  sc->cycleMeasurements = nullptr;
  sc->cycleTrajectories = nullptr;
  sc->cycleMeasurementsMutex = nullptr;
  // Release file management system
  sc->fms = nullptr;
  sp->fms = nullptr;
  // Release main components
  sc->platform->scene = nullptr;
  sc->platform = nullptr;
  sp->mSurvey->scanner = nullptr;
  sp->mSurvey = nullptr;
}

}
}
