#pragma once
#ifdef PYTHON_BINDING

#include <string>
#include <Leg.h>
#include <SurveyPlayback.h>
#include <memory>
#include <noise/RandomnessGenerator.h>
#include <PyPlatformWrapper.h>
#include <PySceneWrapper.h>
#include <PyHeliosOutputWrapper.h>
#include <PyHeliosException.h>
#include <XmlSurveyLoader.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * Helios++ simulation wrapped to be used from Python
 */
class PyHeliosSimulation{
private:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    std::shared_ptr<XmlSurveyLoader> xmlreader = nullptr;
    bool started = false;
    bool paused = false;
    bool stopped = false;
    bool finished = false;
    size_t numThreads = 0;
    size_t callbackFrequency = 0;
    std::string surveyPath = "NULL";
    std::string assetsPath = "NULL";
    std::string outputPath = "NULL";
    std::shared_ptr<Survey> survey = nullptr;
    std::shared_ptr<SurveyPlayback> playback = nullptr;
    boost::thread * thread = nullptr;
    std::shared_ptr<PySimulationCycleCallback> callback = nullptr;
    std::string fixedGpsTimeStart = "";
    bool lasOutput = false;
    bool las10     = false;
    bool zipOutput = false;
    std::shared_ptr<PulseThreadPoolInterface> pulseThreadPool;
    int kdtFactory = 4;
    size_t kdtJobs = 0;
    size_t kdtSAHLossNodes = 32;
    int parallelizationStrategy = 1;
    int chunkSize = 32;
    int warehouseFactor = 1;
public:
    bool finalOutput = true;
    bool exportToFile = true;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyHeliosSimulation() = default;
    /**
     * @brief Build a PyHeliosSimulation instance
     *
     * @param[in] surveyPath Path to the survey XML file
     * @param[in] assetsPath Path to the assets directory
     *
     * @see PyHeliosSimulation::loadSurvey(
     * std::string, std::string, bool, bool)
     */
    PyHeliosSimulation(
        std::string surveyPath,
        std::string assetsPath = "assets/",
        std::string outputPath = "output/",
        size_t numThreads = 0,
        bool lasOutput = false,
        bool las10     = false,
        bool zipOutput = false,
        int kdtFactory = 4,
        size_t kdtJobs = 0,
        size_t kdtSAHLossNodes = 32,
        int parallelizationStrategy = 1,
        int chunkSize = 32,
        int warehouseFactor = 1
    );
    virtual ~PyHeliosSimulation();

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Check if the simulation has been started or not
     *
     * @return True if the simulation has started, false otherwise
     */
    bool isStarted() {return started;}
    /**
     * @brief Check if the simulation has been paused or not
     *
     * @return True if the simulation has been paused, false otherwise
     */
    bool isPaused() {return paused;}
    /**
     * @brief Check if the simulation has been stopped or not
     *
     * @return True if the simulation has been stopped, false otherwise
     */
    bool isStopped() {return stopped;}
    /**
     * @brief Check if the simulation has finished or not
     *
     * @return True if the simulation has finished, false otherwise
     */
    bool isFinished();
    /**
     * @brief Check if the simulation is running or not
     *
     * @return True if the simulation is running, false otherwise
     */
    bool isRunning();
    /**
     * @brief Obtain the survey path used by the simulation
     *
     * @return Survey path used by the simulation
     */
    std::string getSurveyPath() {return surveyPath;}
    /**
     * @brief Obtain the path to assets directory used by the simulation
     *
     * @return Path to the assets directory used by the simulation
     */
    std::string getAssetsPath() {return assetsPath;}

    /**
     * @brief Obtain the survey used by the simulation
     *
     * @return Survey used by the simulation
     */
    Survey & getSurvey() {return *survey;}
    /**
     * @brief Obtain the scanner used by the simulation
     *
     * @return Scanner used by the simulation
     */
    Scanner & getScanner() {return *survey->scanner;}
    /**
     * @brief Obtain the platform used by the simulation
     *
     * @return Platform used by the simulation
     */
    PyPlatformWrapper * getPlatform()
        {return new PyPlatformWrapper(*survey->scanner->platform);}
    PySceneWrapper * getScene()
        {return new PySceneWrapper(*survey->scanner->platform->scene);}
    /**
     * @brief Obtain the number of legs
     *
     * @return Number of legs
     */
    int getNumLegs() {return survey->legs.size();}
    /**
     * @brief Obtain leg at given index
     *
     * @return Leg at given index
     */
    Leg & getLeg(int index) {return *(survey->legs[index]);}
    /**
     * @brief Remove leg at given index
     *
     * @return Leg at given index
     */
    void removeLeg(int index)
        {survey->legs.erase(survey->legs.begin() + index);}
    /**
     * @brief Create a new empty leg
     *
     * @return Created empty leg
     */
    Leg & newLeg(int index);
    /**
     * @brief Obtain callback frequency
     *
     * @return Callback frequency
     */
    size_t getCallbackFrequency() {return callbackFrequency;}
    /**
     * @brief Obtain the number of threads
     *
     * @return Number of threads
     */
    size_t getNumThreads() {return this->numThreads;}
    /**
     * @brief Set the number of threads
     */
    void setNumThreads(size_t numThreads) {this->numThreads = numThreads;}
    /**
     * @brief Set the callback frequency
     */
    void setCallbackFrequency(size_t const callbackFrequency)
    {this->callbackFrequency = callbackFrequency;}
    /**
     * @brief Set the simulation callback to specified python object functor
     */
    void setCallback(PyObject * pyCallback);
    /**
     * @brief Clear simulation callback so it will no longer be invoked
     */
    void clearCallback(){
        playback->callback = nullptr;
        survey->scanner->cycleMeasurements = nullptr;
        survey->scanner->cycleMeasurementsMutex = nullptr;
    }
    std::string getFixedGpsTimeStart(){return fixedGpsTimeStart;}
    void setFixedGpsTimeStart(std::string const fixedGpsTimeStart)
    {this->fixedGpsTimeStart = fixedGpsTimeStart;}
    double getLasOutput(){return lasOutput;}
    void setLasOutput(double lasOutput_){
        if(started) throw PyHeliosException(
            "Cannot modify LAS output flag for already started simulations."
        );
        this->lasOutput = lasOutput_;
    }

    double getLas10(){return las10;}
    void setLas10(double las10_){
        if(started) throw PyHeliosException(
            "Cannot modify LAS v1.0 output flag for already started "
            "simulations."
        );
        this->las10 = las10_;
    }

    double getZipOutput(){return zipOutput;}
    void setZipOutput(bool zipOutput_){
        if(started) throw PyHeliosException(
            "Cannot modify ZIP output flag for already started simulations."
        );
        this->zipOutput = zipOutput_;
    }

    int getKDTFactory(){return kdtFactory;}
    void setKDTFactory(int kdtFactory){
        if(started) throw PyHeliosException(
            "Cannot modify KDT factory for already started simulations."
        );
        this->kdtFactory = kdtFactory;
    }

    size_t getKDTJobs(){return kdtJobs;}
    void setKDTJobs(size_t kdtJobs){
        if(started) throw PyHeliosException(
            "Cannot modify KDT jobs for already started simulations."
        );
        this->kdtJobs = kdtJobs;
    }

    size_t getKDTSAHLossNodes(){return kdtSAHLossNodes;}
    void setKDTSAHLossNodes(size_t kdtSAHLossNodes){
        if(started) throw PyHeliosException(
            "Cannot modify KDT SAH loss nodes for already started simulations."
        );
        this->kdtSAHLossNodes = kdtSAHLossNodes;
    }

    int getParallelizationStrategy(){return parallelizationStrategy;}
    void setParallelizationStrategy(int parallelizationStrategy){
        if(started) throw PyHeliosException(
            "Cannot modify parallelization strategy for already started "
            "simulations."
        );
        this->parallelizationStrategy = parallelizationStrategy;
    }

    int getChunkSize(){return chunkSize;}
    void setChunkSize(int chunkSize){
        if(started) throw PyHeliosException(
            "Cannot modify chunk size for already started simulations."
        );
        this->chunkSize = chunkSize;
    }

    int getWarehouseFactor(){return warehouseFactor;}
    void setWarehouseFactor(int warehouseFactor){
        if(started) throw PyHeliosException(
            "Cannot modify warehouse factor for already started simulations."
        );
        this->warehouseFactor = warehouseFactor;
    }



    // ***  CONTROL FUNCTIONS  *** //
    // *************************** //
    /**
     * @brief Start the simulation if possible. Otherwise, PyHeliosException
     * will be thrown.
     */
    void start();
    /**
     * @brief Pause the simulation if possible. Otherwise, PyHeliosException
     * will be thrown.
     */
    void pause();
    /**
     * @brief Stop the simulation if possible. Otherwise, PyHeliosException
     * will be thrown.
     */
    void stop();
    /**
     * @brief Resume the simulation if possible. Otherwise, PyHeliosException
     * will be thrown.
     */
    void resume();
    /**
     * @brief Cause caller thread to wait until simulation has finished
     */
    PyHeliosOutputWrapper * join();

    // ***  SIMULATION CONFIGURATION FUNCTIONS  *** //
    // ******************************************** //
    /**
     * @brief Load a survey XML file
     *
     * @param[in] legNoiseDisabled True to disable leg noise, False to
     * enable it
     * @param[in] rebuildScene True to force scene rebuild even when a previous
     * scene has been built, False to allow usage of previously built scene
     * when it is available
     */
    void loadSurvey(
        bool legNoiseDisabled = false,
        bool rebuildScene = false,
        bool writeWaveform = false,
        bool calcEchowidth = false,
        bool fullWaveNoise = false,
        bool platformNoiseDisabled = true
    );
    void addRotateFilter(
        double q0,
        double q1,
        double q2,
        double q3,
        std::string partId
    );
    void addScaleFilter(double scaleFactor, std::string partId);
    void addTranslateFilter(double x, double y, double z, std::string partId);
    /**
     * @brief Build the pulse thread pool to be used by the simulation
     * @see PyHeliosSimulation::pulseThreadPool
     * @see Simulation
     */
    void buildPulseThreadPool();

    // ***  SIMULATION COPY  *** //
    // ************************* //
    PyHeliosSimulation * copy();
};

}

#endif
