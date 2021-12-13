#ifdef PYTHON_BINDING

#include <PyHeliosSimulation.h>
#include <PyHeliosException.h>
#include <AbstractDetector.h>
#include <Rotation.h>
#include <RotateFilter.h>
#include <PyHeliosOutputWrapper.h>
#include <chrono>
#include <PulseThreadPoolFactory.h>

using pyhelios::PyHeliosSimulation;
using pyhelios::PyHeliosOutputWrapper;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
PyHeliosSimulation::PyHeliosSimulation(
    std::string surveyPath,
    std::string assetsPath,
    std::string outputPath,
    size_t numThreads,
    bool lasOutput,
    bool las10,
    bool zipOutput,
    int kdtFactory,
    size_t kdtJobs,
    size_t kdtSAHLossNodes,
    int parallelizationStrategy,
    int chunkSize,
    int warehouseFactor
){
    this->lasOutput = lasOutput;
    this->las10 = las10;
    this->zipOutput = zipOutput;
    this->surveyPath = surveyPath;
    this->assetsPath = assetsPath;
    this->outputPath = outputPath;
    if(numThreads == 0) this->numThreads = std::thread::hardware_concurrency();
    else this->numThreads = numThreads;
    this->kdtFactory = kdtFactory;
    if(kdtJobs == 0) this->kdtJobs = std::thread::hardware_concurrency();
    else this->kdtJobs = kdtJobs;
    this->kdtSAHLossNodes = kdtSAHLossNodes;
    this->parallelizationStrategy = parallelizationStrategy;
    this->chunkSize = chunkSize;
    this->warehouseFactor = warehouseFactor;
    xmlreader = std::make_shared<XmlSurveyLoader>(surveyPath, assetsPath);
}
PyHeliosSimulation::~PyHeliosSimulation() {
    if(thread != nullptr) delete thread;
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
Leg & PyHeliosSimulation::newLeg(int index){
    int n = (int) survey->legs.size();
    if(index<0 || index>n) index = n;
    std::shared_ptr<Leg> leg = std::make_shared<Leg>();
    leg->mScannerSettings =
        std::make_shared<ScannerSettings>();
    leg->mPlatformSettings =
        std::make_shared<PlatformSettings>();
    survey->addLeg(index, leg);
    return *leg;
}

// ***  CONTROL FUNCTIONS *** //
// ************************** //
void PyHeliosSimulation::start (){
    if(started) throw PyHeliosException(
        "PyHeliosSimulation was already started so it cannot be started again"
    );

    if(finalOutput){
        survey->scanner->allMeasurements =
            std::make_shared<std::vector<Measurement>>(
                std::vector<Measurement>(0)
            );
        survey->scanner->allTrajectories =
            std::make_shared<std::vector<Trajectory>>(
                std::vector<Trajectory>(0)
            );
        survey->scanner->allMeasurementsMutex = std::make_shared<std::mutex>();
    }

    survey->scanner->detector->lasOutput = lasOutput;
    survey->scanner->detector->las10 = las10;
    survey->scanner->detector->zipOutput = zipOutput;

    buildPulseThreadPool();
    std::cout << "SurveyPlayback built with outputpath:\n'"<<outputPath<<"'"<<std::endl;  // TODO Remove
    playback = std::shared_ptr<SurveyPlayback>(
        new SurveyPlayback(
            survey,
            outputPath,
            parallelizationStrategy,
            pulseThreadPool,
            chunkSize,
            lasOutput,
            las10,
            zipOutput,
            exportToFile
        )
    );
    playback->callback = callback;
    playback->setSimFrequency(simFrequency);
    thread = new boost::thread(
        boost::bind(&SurveyPlayback::start, &(*playback))
    );

    started = true;
}
void PyHeliosSimulation::pause (){
    if(!started) throw PyHeliosException(
        "PyHeliosSimulation was not started so it cannot be paused"
    );
    if(stopped) throw PyHeliosException(
        "PyHeliosSimulation was stopped so it cannot be paused"
    );
    if(finished) throw PyHeliosException(
        "PyHeliosSimulation has finished so it cannot be paused"
    );

    playback->pause(true);

    paused = true;
}
void PyHeliosSimulation::stop (){
    if(!started) throw PyHeliosException(
        "PyHeliosSimulation was not started so it cannot be stopped"
    );
    if(stopped) throw PyHeliosException(
        "PyHeliosSimulation was already stopped so it cannot be stopped again"
    );
    if(finished) throw PyHeliosException(
        "PyHeliosSimulation has finished so it cannot be stopped"
    );

    playback->stop();

    stopped = true;
}
void PyHeliosSimulation::resume (){
    if(!started) throw PyHeliosException(
        "PyHeliosSimulation was not started so it cannot be resumed"
    );
    if(stopped) throw PyHeliosException(
        "PyHeliosSimulation was stopped so it cannot be resumed"
    );
    if(playback->finished) throw PyHeliosException(
        "PyHeliosSimulation has finished so it cannot be resumed"
    );
    if(!paused) throw PyHeliosException(
        "PyHeliosSimulation is not paused so it cannot be resumed"
    );

    playback->pause(false);

    paused = false;
}


bool PyHeliosSimulation::isFinished() {
    return playback->finished;
}

bool PyHeliosSimulation::isRunning() {
    return started && !paused && !stopped && !playback->finished;
}


PyHeliosOutputWrapper * PyHeliosSimulation::join(){
    // Status control
    if(!started || paused) throw PyHeliosException(
        "PyHeliosSimulation is not running so it cannot be joined"
    );

    // Callback concurrency handling (NON BLOCKING MODE)
    if(simFrequency != 0 && callback != nullptr){
        if(!playback->finished) {
            std::vector<Measurement> measurements(0);
            std::vector<Trajectory> trajectories(0);
            return new PyHeliosOutputWrapper(
                measurements,
                trajectories,
                survey->scanner->detector->outputFilePath.string(),
                false
            );
        }
        else{
            finished = true;
            return new PyHeliosOutputWrapper(
                survey->scanner->allMeasurements,
                survey->scanner->allTrajectories,
                survey->scanner->detector->outputFilePath.string(),
                true
            );
        }
    }

    // Join (BLOCKING MODE)
    thread->join();
    finished = true;

    // Final output (BLOCKING MODE)
    if(!finalOutput) return nullptr;
    return new PyHeliosOutputWrapper(
        survey->scanner->allMeasurements,
        survey->scanner->allTrajectories,
        survey->scanner->detector->outputFilePath.string(),
        true
    );
}

// ***  SIMULATION CONFIGURATION FUNCTIONS  *** //
// ******************************************** //
void PyHeliosSimulation::loadSurvey(
    bool legNoiseDisabled,
    bool rebuildScene,
    bool writeWaveform,
    bool calcEchowidth,
    bool fullWaveNoise,
    bool platformNoiseDisabled
){
    xmlreader->sceneLoader.kdtFactoryType = kdtFactory;
    xmlreader->sceneLoader.kdtNumJobs = kdtJobs;
    xmlreader->sceneLoader.kdtSAHLossNodes = kdtSAHLossNodes;
    survey = xmlreader->load(legNoiseDisabled, rebuildScene);
    survey->scanner->setWriteWaveform(writeWaveform);
    survey->scanner->setCalcEchowidth(calcEchowidth);
    survey->scanner->setFullWaveNoise(fullWaveNoise);
    survey->scanner->setPlatformNoiseDisabled(platformNoiseDisabled);
}

void PyHeliosSimulation::addRotateFilter(
    double q0, double q1, double q2, double q3, std::string partId
){
    RotateFilter rf(nullptr);
    delete rf.primsOut;
    rf.primsOut = nullptr;
    rf.useLocalRotation = true;
    rf.localRotation = Rotation(q0, q1, q2, q3, true);
    xmlreader->sceneLoader.sceneSpec.rotations.push_back(rf);
    xmlreader->sceneLoader.sceneSpec.rotationsId.push_back(partId);
}
void PyHeliosSimulation::addScaleFilter(
    double scaleFactor, std::string partId
){
    ScaleFilter sf(nullptr);
    delete sf.primsOut;
    sf.primsOut = nullptr;
    sf.useLocalScaleFactor = true;
    sf.localScaleFactor = scaleFactor;
    xmlreader->sceneLoader.sceneSpec.scales.push_back(sf);
    xmlreader->sceneLoader.sceneSpec.scalesId.push_back(partId);
}
void PyHeliosSimulation::addTranslateFilter(
    double x, double y, double z, std::string partId
){
    TranslateFilter tf(nullptr);
    delete tf.primsOut;
    tf.primsOut = nullptr;
    tf.useLocalTranslation = true;
    tf.localTranslation = glm::dvec3(x, y, z);
    xmlreader->sceneLoader.sceneSpec.translations.push_back(tf);
    xmlreader->sceneLoader.sceneSpec.translationsId.push_back(partId);
}

void PyHeliosSimulation::buildPulseThreadPool(){
    // Prepare pulse thread pool factory
    PulseThreadPoolFactory ptpf(
        parallelizationStrategy,
        numThreads-1,
        survey->scanner->detector->cfg_device_accuracy_m,
        chunkSize,
        warehouseFactor
    );

    // Build pulse thread pool
    pulseThreadPool = ptpf.makePulseThreadPool();
}

// ***  SIMULATION COPY  *** //
// ************************* //
PyHeliosSimulation * PyHeliosSimulation::copy(){
    PyHeliosSimulation *phs = new PyHeliosSimulation(); // The copy itself
    phs->xmlreader = std::make_shared<XmlSurveyLoader>(surveyPath, assetsPath);
    phs->surveyPath = this->surveyPath;
    phs->assetsPath = this->assetsPath;
    phs->outputPath = this->outputPath;
    phs->numThreads = this->numThreads;
    phs->finalOutput = this->finalOutput;
    phs->survey = std::make_shared<Survey>(*survey);
    phs->callback = this->callback;
    phs->lasOutput = this->lasOutput;
    phs->las10     = this->las10;
    phs->zipOutput = this->zipOutput;
    phs->exportToFile = this->exportToFile;
    phs->setSimFrequency(getSimFrequency());
    return phs;
}

// ***  GETTERS and SETTERS  *** //
// ***************************** //
void PyHeliosSimulation::setCallback(PyObject * pyCallback){
    callback = std::make_shared<PySimulationCycleCallback>(
        PySimulationCycleCallback(pyCallback)
    );

    if(survey->scanner->cycleMeasurements == nullptr) {
        survey->scanner->cycleMeasurements =
            std::make_shared<std::vector<Measurement>>(
                std::vector<Measurement>(0)
            );
    }
    if(survey->scanner->cycleTrajectories == nullptr){
        survey->scanner->cycleTrajectories =
            std::make_shared<std::vector<Trajectory>>(
                std::vector<Trajectory>(0)
            );
    }
    if(survey->scanner->cycleMeasurementsMutex == nullptr){
        survey->scanner->cycleMeasurementsMutex =
            std::make_shared<std::mutex>();
    }
}

#endif