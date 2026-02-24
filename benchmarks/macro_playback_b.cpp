/*
This benchmark is a macro benchmark for an entire SurveyPlayback.
It follows the setup of a SurveyPlayback in LidarSim.cpp with the tls_toyblocks.
The benchmark tests the performance of the SurveyPlayback->start() method.
*/

#include <benchmark/benchmark.h>

#include "Survey.h"
#include <assetloading/XmlSurveyLoader.h>
#include <boost/filesystem.hpp>
#include <filems/facade/FMSFacade.h>
#include <filems/factory/FMSFacadeFactory.h>
#include <main/LidarSim.h>
#include <memory>
#include <noise/RandomnessGenerator.h>
#include <scanner/detector/PulseThreadPoolFactory.h>

#include "logging.hpp"

namespace fs = boost::filesystem;

bool logging::LOGGING_SHOW_TRACE, logging::LOGGING_SHOW_DEBUG,
  logging::LOGGING_SHOW_INFO, logging::LOGGING_SHOW_TIME,
  logging::LOGGING_SHOW_WARN, logging::LOGGING_SHOW_ERR;

static void
macro_playback_benchmark(benchmark::State& state)
{
  // manually set up a LidarSim
  fs::path src_file = fs::path(__FILE__).parent_path(); // .../benchmarks
  fs::path repo_root = src_file.parent_path();          // repo root
  std::string surveyPath =
    (repo_root / "data" / "surveys" / "toyblocks" / "tls_toyblocks.xml")
      .string();
  std::vector<std::string> assetsPath = {
    repo_root.string(),
    (repo_root / "python" / "helios").string(),
    (repo_root / "assets").string(),
  };
  std::string outputPath = (repo_root / "output").string();
  bool writeWaveform = false;
  bool writePulse = false;
  bool calcEchowidth = false;
  int parallelizationStrategy = 0;
  size_t njobs = 0UL;
  int chunkSize = 32;
  int warehouseFactor = 4;
  bool fullWaveNoise = false;
  bool splitByChannel = true;
  bool platformNoiseDisabled = false;
  bool legNoiseDisabled = false;
  bool rebuildScene = false;
  bool writeScene = true;
  bool lasOutput = true;
  bool las10 = false;
  bool zipOutput = false;
  bool fixedIncidenceAngle = false;
  std::string gpsStartTime = "";
  double lasScale = (1.0E-4);
  ;
  int kdtType = 4;
  size_t kdtJobs = 1UL;
  size_t kdtGeomJobs = 1UL;
  size_t sahLossNodes = 32;
  ;
  bool const legacyEnergyModel = true;

  std::shared_ptr<XmlSurveyLoader> xmlreader =
    std::make_shared<XmlSurveyLoader>(surveyPath, assetsPath);
  xmlreader->sceneLoader.kdtFactoryType = kdtType;
  xmlreader->sceneLoader.kdtNumJobs = kdtJobs;
  xmlreader->sceneLoader.kdtGeomJobs = kdtGeomJobs;
  xmlreader->sceneLoader.kdtSAHLossNodes = sahLossNodes;
  std::shared_ptr<Survey> survey = xmlreader->load(legNoiseDisabled);
  if (survey == nullptr) {
    std::cerr << "Error loading survey from XML file: " << surveyPath
              << std::endl;
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
  std::shared_ptr<helios::filems::FMSFacade> fms =
    helios::filems::FMSFacadeFactory().buildFacade(outputPath,
                                                   lasScale,
                                                   lasOutput,
                                                   las10,
                                                   zipOutput,
                                                   splitByChannel,
                                                   *survey);

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
                                     parallelizationStrategy,
                                     pulseThreadPool,
                                     std::abs(chunkSize),
                                     gpsStartTime,
                                     legacyEnergyModel,
                                     true,
                                     false,
                                     fms);

  // Start simulation
  TimeWatcher tw;
  tw.start();

  for (auto _ : state) {
    playback->start();
  }

  tw.stop();
  tw.reportFormat("Total simulation time: ");

  // Disconnect FMS
  fms->disconnect();

  // Release resources before finishing
  // Release scanner
  std::shared_ptr<Scanner> sc = playback->mSurvey->scanner;
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
  playback->fms = nullptr;
  // Release main components
  sc->platform->scene = nullptr;
  sc->platform = nullptr;
  playback->mSurvey->scanner = nullptr;
  playback->mSurvey = nullptr;
}
BENCHMARK(macro_playback_benchmark);
BENCHMARK_MAIN();
