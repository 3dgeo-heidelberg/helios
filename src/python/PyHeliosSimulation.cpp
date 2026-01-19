#include <AbstractDetector.h>
#include <HeliosException.h>
#include <PyHeliosSimulation.h>
#include <RotateFilter.h>
#include <Rotation.h>

#include <PulseThreadPoolFactory.h>
#include <chrono>
#include <filems/facade/FMSFacade.h>
#include <filems/facade/FMSWriteFacade.h>
#include <filems/factory/FMSFacadeFactory.h>
// #include <PyScanningStripWrapper.h>

using helios::filems::FMSWriteFacade;

namespace fms = helios::filems;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
PyHeliosSimulation::PyHeliosSimulation(
  std::string surveyPath,
  const std::vector<std::string>& assetsPath,
  std::string outputPath,
  size_t numThreads,
  bool lasOutput,
  bool las10,
  bool zipOutput,
  bool splitByChannel,
  int kdtFactory,
  size_t kdtJobs,
  size_t kdtSAHLossNodes,
  int parallelizationStrategy,
  int chunkSize,
  int warehouseFactor)
{
  std::cout << "Constructor called with:" << std::endl;
  std::cout << "surveyPath: " << surveyPath << std::endl;
  std::cout << "outputPath: " << outputPath << std::endl;
  std::cout << "numThreads: " << numThreads << std::endl;
  std::cout << "lasOutput: " << lasOutput << std::endl;
  std::cout << "las10: " << las10 << std::endl;
  std::cout << "zipOutput: " << zipOutput << std::endl;
  std::cout << "splitByChannel: " << splitByChannel << std::endl;
  std::cout << "kdtFactory: " << kdtFactory << std::endl;
  std::cout << "kdtJobs: " << kdtJobs << std::endl;
  std::cout << "kdtSAHLossNodes: " << kdtSAHLossNodes << std::endl;
  std::cout << "parallelizationStrategy: " << parallelizationStrategy
            << std::endl;
  std::cout << "chunkSize: " << chunkSize << std::endl;
  std::cout << "warehouseFactor: " << warehouseFactor << std::endl;

  std::cout << "assetsPath:" << std::endl;
  for (const auto& path : assetsPath) {
    std::cout << path << std::endl;
  }

  this->fixedGpsTimeStart = "";
  this->lasOutput = lasOutput;
  this->las10 = las10;
  this->zipOutput = zipOutput;
  this->splitByChannel = splitByChannel;
  this->surveyPath = surveyPath;
  this->assetsPath = assetsPath;
  this->outputPath = outputPath;
  if (numThreads == 0)
    this->numThreads = std::thread::hardware_concurrency();
  else
    this->numThreads = numThreads;
  this->kdtFactory = kdtFactory;
  if (kdtJobs == 0)
    this->kdtJobs = std::thread::hardware_concurrency();
  else
    this->kdtJobs = kdtJobs;
  this->kdtSAHLossNodes = kdtSAHLossNodes;
  this->parallelizationStrategy = parallelizationStrategy;
  this->chunkSize = chunkSize;
  this->warehouseFactor = warehouseFactor;
  xmlreader = std::make_shared<XmlSurveyLoader>(surveyPath, this->assetsPath);
}
PyHeliosSimulation::~PyHeliosSimulation()
{
  if (survey != nullptr) {
    // Release shared resources
    bool sharedDetector = false;
    bool sharedScene = false;
    std::shared_ptr<AbstractDetector> ad = survey->scanner->getDetector();
    std::shared_ptr<Scene> scene = survey->scanner->platform->scene;
    for (PyHeliosSimulation* copy : copies) {
      if (copy->survey->scanner->getDetector() == ad) {
        sharedDetector = true;
      }
      if (copy->survey->scanner->platform->scene == scene) {
        sharedScene = true;
      }
    }
    if (!sharedDetector) {
      survey->scanner->getDetector()->shutdown();
    }
    if (!sharedScene) {
      survey->scanner->platform->scene->shutdown();
    }
    // Update copy tracking for non-destroyed copies
    for (PyHeliosSimulation* copy : copies) {
      for (size_t i = 0; i < copy->copies.size(); ++i) {
        if (copy->copies[i] == this) {
          copy->copies.erase(copy->copies.begin() + i);
          break;
        }
      }
    }
  }
  if (thread) {
    if (thread->joinable()) {
      thread->join();
    }
    delete thread;
  }
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
Leg&
PyHeliosSimulation::newLeg(int index)
{
  int const n = (int)survey->legs.size();
  if (index < 0 || index > n)
    index = n;
  std::shared_ptr<Leg> leg = std::make_shared<Leg>();
  leg->mScannerSettings = std::make_shared<ScannerSettings>();
  leg->mPlatformSettings = std::make_shared<PlatformSettings>();
  survey->addLeg(index, leg);
  return *leg;
}

std::shared_ptr<ScanningStrip>
PyHeliosSimulation::newScanningStrip(std::string const& stripId)
{
  return std::make_shared<ScanningStrip>(stripId);
}
bool
PyHeliosSimulation::assocLegWithScanningStrip(
  Leg& leg,
  std::shared_ptr<ScanningStrip> strip)
{
  // Check leg status
  bool previouslyAssoc = leg.isContainedInAStrip();
  // Find leg pointer by serial ID
  Leg* _leg = nullptr;
  int const legSerialId = leg.getSerialId();
  for (std::shared_ptr<Leg> l : survey->legs) {
    if (l->getSerialId() == legSerialId) {
      _leg = l.get();
      break;
    }
  }
  // Associate
  leg.setStrip(strip);
  strip->emplace(legSerialId, _leg);
  // Return original leg association status
  return previouslyAssoc;
}

// ***  CONTROL FUNCTIONS *** //
// ************************** //
void
PyHeliosSimulation::start()
{

  if (started)
    throw HeliosException(
      "PyHeliosSimulation was already started so it cannot be started again");

  if (finalOutput) {
    survey->scanner->allOutputPaths =
      std::make_shared<std::vector<std::string>>(std::vector<std::string>(0));
    survey->scanner->allMeasurements =
      std::make_shared<std::vector<Measurement>>(std::vector<Measurement>(0));
    survey->scanner->allTrajectories =
      std::make_shared<std::vector<Trajectory>>(std::vector<Trajectory>(0));
    survey->scanner->allMeasurementsMutex = std::make_shared<std::mutex>();
  }

  std::shared_ptr<fms::FMSFacade> fms =
    exportToFile ? fms::FMSFacadeFactory().buildFacade(outputPath,
                                                       lasScale,
                                                       lasOutput,
                                                       las10,
                                                       zipOutput,
                                                       splitByChannel,
                                                       *survey)
                 : nullptr;

  buildPulseThreadPool();
  playback = std::make_shared<SurveyPlayback>(survey,
                                              fms,
                                              parallelizationStrategy,
                                              pulseThreadPool,
                                              chunkSize,
                                              fixedGpsTimeStart,
                                              legacyEnergyModel,
                                              exportToFile,
                                              true);
  playback->callback = callback;
  playback->setCallbackFrequency(callbackFrequency);
  thread = new std::thread(std::bind(&SurveyPlayback::start, playback));

  started = true;
}
void
PyHeliosSimulation::pause()
{
  if (!started)
    throw HeliosException(
      "PyHeliosSimulation was not started so it cannot be paused");
  if (stopped)
    throw HeliosException(
      "PyHeliosSimulation was stopped so it cannot be paused");
  if (finished)
    throw HeliosException(
      "PyHeliosSimulation has finished so it cannot be paused");

  playback->pause(true);

  paused = true;
}
void
PyHeliosSimulation::stop()
{
  if (!started)
    throw HeliosException(
      "PyHeliosSimulation was not started so it cannot be stopped");
  if (stopped)
    throw HeliosException(
      "PyHeliosSimulation was already stopped so it cannot be stopped again");
  if (finished)
    throw HeliosException(
      "PyHeliosSimulation has finished so it cannot be stopped");

  playback->stop();

  stopped = true;
}
void
PyHeliosSimulation::resume()
{
  if (!started)
    throw HeliosException(
      "PyHeliosSimulation was not started so it cannot be resumed");
  if (stopped)
    throw HeliosException(
      "PyHeliosSimulation was stopped so it cannot be resumed");
  if (playback->finished)
    throw HeliosException(
      "PyHeliosSimulation has finished so it cannot be resumed");
  if (!paused)
    throw HeliosException(
      "PyHeliosSimulation is not paused so it cannot be resumed");

  playback->pause(false);

  paused = false;
}

bool
PyHeliosSimulation::isFinished()
{
  return playback->finished;
}

bool
PyHeliosSimulation::isRunning()
{
  return started && !paused && !stopped && !playback->finished;
}

py::tuple
PyHeliosSimulation::join()
{
  // Status control
  if (!started || paused) {
    throw std::runtime_error(
      "PyHeliosSimulation is not running so it cannot be joined");
  }

  // Obtain measurements output path
  std::string mwOutPath = "";
  if (exportToFile) {
    mwOutPath =
      survey->scanner->fms->write.getMeasurementWriterOutputPath().string();
  }

  // Callback concurrency handling (NON BLOCKING MODE)
  if (callbackFrequency != 0 && callback != nullptr) {
    if (!playback->finished) {
      // Return empty vectors if the simulation is not finished yet
      return py::make_tuple(std::vector<Measurement>{},
                            std::vector<Trajectory>{},
                            mwOutPath,
                            std::vector<std::string>{ mwOutPath },
                            false);
    } else {
      // Return collected data if the simulation is finished
      finished = true;
      return py::make_tuple(*survey->scanner->allMeasurements,
                            *survey->scanner->allTrajectories,
                            mwOutPath,
                            *survey->scanner->allOutputPaths,
                            true);
    }
  }

  // Join (BLOCKING MODE)
  if (thread && thread->joinable()) {
    thread->join();
  }
  if (playback->fms != nullptr) {
    playback->fms->disconnect();
  }
  finished = true;

  // Final output (BLOCKING MODE)
  if (!finalOutput) {
    return py::make_tuple(std::vector<Measurement>{},
                          std::vector<Trajectory>{},
                          mwOutPath,
                          std::vector<std::string>{},
                          false);
  }
  return py::make_tuple(*survey->scanner->allMeasurements,
                        *survey->scanner->allTrajectories,
                        mwOutPath,
                        *survey->scanner->allOutputPaths,
                        true);
}

// ***  SIMULATION CONFIGURATION FUNCTIONS  *** //
// ******************************************** //
void
PyHeliosSimulation::loadSurvey(bool legNoiseDisabled,
                               bool rebuildScene,
                               bool writeWaveform,
                               bool calcEchowidth,
                               bool fullWaveNoise,
                               bool platformNoiseDisabled)
{
  xmlreader->sceneLoader.kdtFactoryType = kdtFactory;
  xmlreader->sceneLoader.kdtNumJobs = kdtJobs;
  xmlreader->sceneLoader.kdtSAHLossNodes = kdtSAHLossNodes;
  survey = xmlreader->load(legNoiseDisabled, rebuildScene);
  survey->scanner->setWriteWaveform(writeWaveform);
  survey->scanner->setCalcEchowidth(calcEchowidth);
  survey->scanner->setFullWaveNoise(fullWaveNoise);
  survey->scanner->setPlatformNoiseDisabled(platformNoiseDisabled);
}

void
PyHeliosSimulation::addRotateFilter(double q0,
                                    double q1,
                                    double q2,
                                    double q3,
                                    std::string partId)
{
  RotateFilter rf(nullptr);
  delete rf.primsOut;
  rf.primsOut = nullptr;
  rf.useLocalRotation = true;
  rf.localRotation = Rotation(q0, q1, q2, q3, true);
  xmlreader->sceneLoader.sceneSpec.rotations.push_back(rf);
  xmlreader->sceneLoader.sceneSpec.rotationsId.push_back(partId);
}
void
PyHeliosSimulation::addScaleFilter(double scaleFactor, std::string partId)
{
  ScaleFilter sf(nullptr);
  delete sf.primsOut;
  sf.primsOut = nullptr;
  sf.useLocalScaleFactor = true;
  sf.localScaleFactor = scaleFactor;
  xmlreader->sceneLoader.sceneSpec.scales.push_back(sf);
  xmlreader->sceneLoader.sceneSpec.scalesId.push_back(partId);
}
void
PyHeliosSimulation::addTranslateFilter(double x,
                                       double y,
                                       double z,
                                       std::string partId)
{
  TranslateFilter tf(nullptr);
  delete tf.primsOut;
  tf.primsOut = nullptr;
  tf.useLocalTranslation = true;
  tf.localTranslation = glm::dvec3(x, y, z);
  xmlreader->sceneLoader.sceneSpec.translations.push_back(tf);
  xmlreader->sceneLoader.sceneSpec.translationsId.push_back(partId);
}

void
PyHeliosSimulation::buildPulseThreadPool()
{
  // Prepare pulse thread pool factory
  PulseThreadPoolFactory ptpf(
    parallelizationStrategy,
    numThreads - 1,
    survey->scanner->getDetector()->cfg_device_accuracy_m,
    chunkSize,
    warehouseFactor);

  // Build pulse thread pool
  pulseThreadPool = ptpf.makePulseThreadPool();
}

// ***  SIMULATION COPY  *** //
// ************************* //
PyHeliosSimulation*
PyHeliosSimulation::copy()
{
  // Copy
  PyHeliosSimulation* phs = new PyHeliosSimulation(); // The copy itself
  phs->xmlreader = std::make_shared<XmlSurveyLoader>(surveyPath, assetsPath);
  phs->surveyPath = this->surveyPath;
  phs->assetsPath = this->assetsPath;
  phs->outputPath = this->outputPath;
  phs->numThreads = this->numThreads;
  phs->finalOutput = this->finalOutput;
  phs->callback = this->callback;
  phs->lasOutput = this->lasOutput;
  phs->las10 = this->las10;
  phs->zipOutput = this->zipOutput;
  phs->exportToFile = this->exportToFile;
  phs->setCallbackFrequency(getCallbackFrequency());
  phs->survey = std::make_shared<Survey>(*survey);
  phs->survey->scanner->initializeSequentialGenerators();
  // Track copies
  phs->copies = copies;
  phs->copies.push_back(this);
  for (PyHeliosSimulation* phsi : copies)
    phsi->copies.push_back(phs);
  copies.push_back(phs);
  // Return
  return phs;
}

void
PyHeliosSimulation::setCallback(pybind11::object pyCallback)
{
  callback = std::make_shared<SimulationCycleCallbackWrap>(pyCallback);

  if (survey->scanner->cycleMeasurements == nullptr) {
    survey->scanner->cycleMeasurements =
      std::make_shared<std::vector<Measurement>>(0);
  }
  if (survey->scanner->cycleTrajectories == nullptr) {
    survey->scanner->cycleTrajectories =
      std::make_shared<std::vector<Trajectory>>(0);
  }
  if (survey->scanner->cycleMeasurementsMutex == nullptr) {
    survey->scanner->cycleMeasurementsMutex = std::make_shared<std::mutex>();
  }
}

// ***  INTERNAL USE  *** //
// ********************** //
std::shared_ptr<DynScene>
PyHeliosSimulation::_getDynScene()
{
  try {
    return std::dynamic_pointer_cast<DynScene>(
      survey->scanner->platform->scene);
  } catch (std::exception& ex) {
    throw HeliosException(
      "Failed to obtain dynamic scene. Current scene is not dynamic.");
  }
}
