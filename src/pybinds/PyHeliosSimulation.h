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
    size_t simFrequency = 0;
    std::string surveyPath = "NULL";
    std::string assetsPath = "NULL";
    std::string outputPath = "NULL";
    std::shared_ptr<Survey> survey = nullptr;
    std::shared_ptr<SurveyPlayback> playback = nullptr;
    boost::thread * thread = nullptr;
    std::shared_ptr<PySimulationCycleCallback> callback = nullptr;
    bool lasOutput = false;
    bool las10     = false;
    bool zipOutput = false;
    std::shared_ptr<PulseThreadPoolInterface> pulseThreadPool;
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
        bool zipOutput = false
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
     * @brief Obtain simulation frequency
     *
     * @return Simulation frequency
     */
    size_t getSimFrequency() {return simFrequency;}
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
     * @brief Set the simulation frequency
     */
    void setSimFrequency(size_t simFrequency)
        {this->simFrequency = simFrequency;}
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
          "Cannot modify LAS v1.0 output flag for already started simulations."
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

    // ***  SIMULATION COPY  *** //
    // ************************* //
    PyHeliosSimulation * copy();
};

}

#endif
