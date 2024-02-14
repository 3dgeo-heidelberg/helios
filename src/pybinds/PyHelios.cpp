#include <boost/python.hpp>
#include <PyHeliosSimulation.h>
#include <PythonDVec3.h>
#include <PyBeamDeflectorWrapper.h>
#include <PyDetectorWrapper.h>
#include <PyIntegerList.h>
#include <PyDoubleVector.h>
#include <PyPlatformWrapper.h>
#include <PyScannerWrapper.h>
#include <PyPrimitiveWrapper.h>
#include <PySimulationCycleCallback.h>
#include <PyScanningStripWrapper.h>
#include <Material.h>
#include <gdal_priv.h>
#include <logging.hpp>

// LOGGING FLAGS (DO NOT MODIFY HERE BUT IN logging.hpp makeDefault())
bool    logging::LOGGING_SHOW_TRACE,    logging::LOGGING_SHOW_DEBUG,
        logging::LOGGING_SHOW_INFO,     logging::LOGGING_SHOW_TIME,
        logging::LOGGING_SHOW_WARN,     logging::LOGGING_SHOW_ERR;

// ***  PYHELIOS PYTHON MODULE  *** //
// ******************************** //
BOOST_PYTHON_MODULE(_pyhelios){
    // Namespace must be used locally to prevent conflicts
    using namespace boost::python;
    using namespace pyhelios;


    // Configure logging system
    logging::makeQuiet();
    logging::configure({
        {"type", "std_out"}
    });

    // Enable GDAL (Load its drivers)
    GDALAllRegister();

    // Definitions
    def(
        "loggingQuiet",
        logging::makeQuiet,
        "Set the logging verbosity level to quiet"
    );
    def(
        "loggingSilent",
        logging::makeSilent,
        "Set the logging verbosity level to silent"
    );
    def(
        "loggingDefault",
        logging::makeDefault,
        "Set the logging verbosity level to default"
    );
    def(
        "loggingVerbose",
        logging::makeVerbose,
        "Set the logging verbosity level to verbose"
    );
    def(
        "loggingVerbose2",
        logging::makeVerbose2,
        "Set the logging verbosity level to verbose 2"
    );
    def(
        "loggingTime",
        logging::makeTime,
        "Set the logging verbosity level to time"
    );

    // Register PyHeliosSimulation
    def(
        "setDefaultRandomnessGeneratorSeed",
        setDefaultRandomnessGeneratorSeed,
        "Set the seed for the default randomness generator"
    );
    class_<PyHeliosSimulation>("Simulation", init<>())
        .def(init<
            std::string,
            std::string,
            std::string,
            size_t,
            bool,
            bool,
            bool,
            bool
        >())
        .def(init<
            std::string,
            std::string,
            std::string,
            size_t,
            bool,
            bool,
            bool,
            bool,
            int,
            size_t,
            size_t,
            int,
            int,
            int
        >())
        .def(
            "loadSurvey",
            &PyHeliosSimulation::loadSurvey
        )
        .def(
            "copy",
            &PyHeliosSimulation::copy,
            return_value_policy<manage_new_object>()
        )
        .def("isStarted", &PyHeliosSimulation::isStarted)
        .def("isPaused", &PyHeliosSimulation::isPaused)
        .def("isStopped", &PyHeliosSimulation::isStopped)
        .def("isFinished", &PyHeliosSimulation::isFinished)
        .def("isRunning", &PyHeliosSimulation::isRunning)
        .def("getSurveyPath", &PyHeliosSimulation::getSurveyPath)
        .def("getAssetsPath", &PyHeliosSimulation::getAssetsPath)
        .def(
            "getSurvey",
            &PyHeliosSimulation::getSurvey,
            return_internal_reference<>()
        )
        .def(
            "getScanner",
            &PyHeliosSimulation::getScanner,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getPlatform",
            &PyHeliosSimulation::getPlatform,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getScene",
            &PyHeliosSimulation::getScene,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getNumLegs",
            &PyHeliosSimulation::getNumLegs
        )
        .def(
            "getLeg",
            &PyHeliosSimulation::getLeg,
            return_internal_reference<>()
        )
        .def(
            "removeLeg",
            &PyHeliosSimulation::removeLeg
        )
        .def(
            "newLeg",
            &PyHeliosSimulation::newLeg,
            return_internal_reference<>()
        )
        .def(
            "newScanningStrip",
            &PyHeliosSimulation::newScanningStrip,
            return_value_policy<manage_new_object>()
        )
        .def(
            "assocLegWithScanningStrip",
            &PyHeliosSimulation::assocLegWithScanningStrip
        )
        .add_property(
            "simulationFrequency",
            &PyHeliosSimulation::getSimFrequency,
            &PyHeliosSimulation::setSimFrequency
        )
        .add_property(
            "dynSceneStep",
            &PyHeliosSimulation::getDynSceneStep,
            &PyHeliosSimulation::setDynSceneStep
        )
        .add_property(
            "callbackFrequency",
            &PyHeliosSimulation::getCallbackFrequency,
            &PyHeliosSimulation::setCallbackFrequency
        )
        .add_property(
            "finalOutput",
            &PyHeliosSimulation::finalOutput,
            &PyHeliosSimulation::finalOutput
        )
        .add_property(
            "legacyEnergyModel",
            &PyHeliosSimulation::legacyEnergyModel,
            &PyHeliosSimulation::legacyEnergyModel
        )
        .add_property(
            "exportToFile",
            &PyHeliosSimulation::exportToFile,
            &PyHeliosSimulation::exportToFile
        )
        .def("start", &PyHeliosSimulation::start)
        .def("pause", &PyHeliosSimulation::pause)
        .def("stop", &PyHeliosSimulation::stop)
        .def("resume", &PyHeliosSimulation::resume)
        .def(
            "join",
            &PyHeliosSimulation::join,
            return_value_policy<manage_new_object>()
        )
        .def("setCallback", &PyHeliosSimulation::setCallback)
        .def("clearCallback", &PyHeliosSimulation::clearCallback)
        .add_property(
            "fixedGpsTimeStart",
            &PyHeliosSimulation::getFixedGpsTimeStart,
            &PyHeliosSimulation::setFixedGpsTimeStart
        )
        .add_property(
            "lasOutput",
            &PyHeliosSimulation::getLasOutput,
            &PyHeliosSimulation::setLasOutput
        )
        .add_property(
            "las10",
            &PyHeliosSimulation::getLas10,
            &PyHeliosSimulation::setLas10
        )
        .add_property(
            "zipOutput",
            &PyHeliosSimulation::getZipOutput,
            &PyHeliosSimulation::setZipOutput
        )
        .add_property(
            "splitByChannel",
            &PyHeliosSimulation::getSplitByChannel,
            &PyHeliosSimulation::setSplitByChannel
        )
        .add_property(
            "lasScale",
            &PyHeliosSimulation::getLasScale,
            &PyHeliosSimulation::setLasScale
        )
        .add_property(
            "numThreads",
            &PyHeliosSimulation::getNumThreads,
            &PyHeliosSimulation::setNumThreads
        )
        .add_property(
            "kdtFactory",
            &PyHeliosSimulation::getKDTFactory,
            &PyHeliosSimulation::setKDTFactory
        )
        .add_property(
            "kdtJobs",
            &PyHeliosSimulation::getKDTJobs,
            &PyHeliosSimulation::setKDTJobs
        )
        .add_property(
            "kdtSAHLossNodes",
            &PyHeliosSimulation::getKDTSAHLossNodes,
            &PyHeliosSimulation::setKDTSAHLossNodes
        )
        .add_property(
            "parallelizationStrategy",
            &PyHeliosSimulation::getParallelizationStrategy,
            &PyHeliosSimulation::setParallelizationStrategy
        )
        .add_property(
            "chunkSize",
            &PyHeliosSimulation::getChunkSize,
            &PyHeliosSimulation::setChunkSize
        )
        .add_property(
            "warehouseFactor",
            &PyHeliosSimulation::getWarehouseFactor,
            &PyHeliosSimulation::setWarehouseFactor
        )
        .def(
            "addRotateFilter",
            &PyHeliosSimulation::addRotateFilter
        )
        .def(
            "addScaleFilter",
            &PyHeliosSimulation::addScaleFilter
        )
        .def(
            "addTranslateFilter",
            &PyHeliosSimulation::addTranslateFilter
        )
        ;

    // Register Survey
    class_<Survey, boost::noncopyable>("Survey", no_init)
        .def("calculateLength", &Survey::calculateLength)
        .def("getLength", &Survey::getLength)
        .add_property("name", &Survey::name, &Survey::name)
        .add_property("numRuns", &Survey::numRuns, &Survey::numRuns)
        .add_property(
            "simSpeedFactor",
            &Survey::simSpeedFactor,
            &Survey::simSpeedFactor
        )
    ;

    // Register ScannerSettings
    class_<ScannerSettings>("ScannerSettings", no_init)
        .add_property(
            "active",
            &ScannerSettings::active,
            &ScannerSettings::active
        )
        .add_property(
            "headRotatePerSec",
            &ScannerSettings::headRotatePerSec_rad,
            &ScannerSettings::headRotatePerSec_rad
        )
        .add_property(
            "headRotateStart",
            &ScannerSettings::headRotateStart_rad,
            &ScannerSettings::headRotateStart_rad
        )
        .add_property(
            "headRotateStop",
            &ScannerSettings::headRotateStop_rad,
            &ScannerSettings::headRotateStop_rad
        )
        .add_property(
            "pulseFreq",
            &ScannerSettings::pulseFreq_Hz,
            &ScannerSettings::pulseFreq_Hz
        )
        .add_property(
            "scanAngle",
            &ScannerSettings::scanAngle_rad,
            &ScannerSettings::scanAngle_rad
        )
        .add_property(
            "verticalAngleMin",
            &ScannerSettings::verticalAngleMin_rad,
            &ScannerSettings::verticalAngleMin_rad
        )
        .add_property(
            "verticalAngleMax",
            &ScannerSettings::verticalAngleMax_rad,
            &ScannerSettings::verticalAngleMax_rad
        )
        .add_property(
            "scanFreq",
            &ScannerSettings::scanFreq_Hz,
            &ScannerSettings::scanFreq_Hz
        )
        .add_property(
            "beamDivAngle",
            &ScannerSettings::beamDivAngle,
            &ScannerSettings::beamDivAngle
        )
        .add_property(
            "trajectoryTimeInterval",
            &ScannerSettings::trajectoryTimeInterval,
            &ScannerSettings::trajectoryTimeInterval
        )
        .add_property(
            "id",
            &ScannerSettings::id,
            &ScannerSettings::id
        )
        .def(
            "hasTemplate",
            &ScannerSettings::hasTemplate
        )
        .def(
            "getTemplate",
            &ScannerSettings::getTemplate,
            return_internal_reference<>()
        )
        .def(
            "toString",
            &ScannerSettings::toString
        )
    ;

    // Register PlatformSettings
    class_<PlatformSettings>("PlatformSettings", no_init)
        .add_property("id", &PlatformSettings::id, &PlatformSettings::id)
        .add_property("x", &PlatformSettings::x, &PlatformSettings::x)
        .add_property("y", &PlatformSettings::y, &PlatformSettings::y)
        .add_property("z", &PlatformSettings::z, &PlatformSettings::z)
        .add_property(
            "onGround",
            &PlatformSettings::onGround,
            &PlatformSettings::onGround
        )
        .add_property(
            "stopAndTurn",
            &PlatformSettings::stopAndTurn,
            &PlatformSettings::stopAndTurn
        )
        .add_property(
            "movePerSec",
            &PlatformSettings::movePerSec_m,
            &PlatformSettings::movePerSec_m
        )
        .add_property(
            "slowdownEnabled",
            &PlatformSettings::slowdownEnabled,
            &PlatformSettings::slowdownEnabled
        )
        .add_property(
            "yawAtDepartureSpecified",
            &PlatformSettings::yawAtDepartureSpecified,
            &PlatformSettings::yawAtDeparture
        )
        .add_property(
            "yawAtDeparture",
            &PlatformSettings::yawAtDeparture,
            &PlatformSettings::yawAtDeparture
        )
        .add_property(
            "smoothTurn",
            &PlatformSettings::smoothTurn,
            &PlatformSettings::smoothTurn
        )
        .def(
            "hasTemplate",
            &PlatformSettings::hasTemplate
        )
        .def(
            "getTemplate",
            &PlatformSettings::getTemplate,
            return_internal_reference<>()
        )
        .def(
            "toString",
            &PlatformSettings::toString
        )
    ;

    // Register Leg
    class_<Leg, boost::noncopyable>("Leg", no_init)
        .add_property("length", &Leg::getLength, &Leg::setLength)
        .add_property("serialId", &Leg::getSerialId, &Leg::setSerialId)
        .add_property(
            "strip",
            make_function(
                +[](const Leg &leg) {
                    return new PyScanningStripWrapper(leg.getStrip());
                },
                return_value_policy<manage_new_object>()
            ),
            +[](Leg& leg, PyScanningStripWrapper *pssw) {
                leg.setStrip(pssw->ss);
            }
        )
        .def(
            "getScannerSettings",
            &Leg::getScannerSettings,
            return_internal_reference<>()
        )
        .def(
            "getPlatformSettings",
            &Leg::getPlatformSettings,
            return_internal_reference<>()
        )
        .def("isContainedInAStrip", &Leg::isContainedInAStrip)
    ;

    // Register ScanningStrip
    class_<PyScanningStripWrapper>("ScanningStrip", no_init)
        .add_property(
            "stripId",
            &PyScanningStripWrapper::getStripId,
            &PyScanningStripWrapper::setStripId
        )
        .def(
            "getLeg",
            &PyScanningStripWrapper::getLegRef,
            return_internal_reference<>()
        )
        .def(
            "isLastLegInStrip",
            &PyScanningStripWrapper::isLastLegInStrip
        )
        .def<bool (PyScanningStripWrapper::*)(int const)>(
            "has", &PyScanningStripWrapper::has
        )
        .def<bool (PyScanningStripWrapper::*)(Leg &)>(
            "has", &PyScanningStripWrapper::has
        )
    ;


    // Register Scanner
    class_<PyScannerWrapper, boost::noncopyable>("Scanner", no_init)
        .add_property(
            "fwfSettings",
            &PyScannerWrapper::getFWFSettings,
            &PyScannerWrapper::setFWFSettings
        )
        .add_property(
            "numTimeBins",
            &PyScannerWrapper::getNumTimeBins,
            &PyScannerWrapper::setNumTimeBins
        )
        .add_property(
            "peakIntensityIndex",
            &PyScannerWrapper::getPeakIntensityIndex,
            &PyScannerWrapper::setPeakIntensityIndex
        )
        .def(
            "getTimeWave",
            &PyScannerWrapper::getTimeWave,
            return_value_policy<manage_new_object>()
        )
        .add_property(
            "pulseFreq_Hz",
            &PyScannerWrapper::getPulseFreq_Hz,
            &PyScannerWrapper::setPulseFreq_Hz
        )
        .add_property(
            "lastPulseWasHit",
            &PyScannerWrapper::lastPulseWasHit,
            static_cast<void(PyScannerWrapper::*)(bool const)>(
                &PyScannerWrapper::setLastPulseWasHit
            )
        )
        .def("toString", &PyScannerWrapper::toString)
        .def(
            "getCurrentPulseNumber",
            static_cast<int(PyScannerWrapper::*)(size_t const) const>(
                &PyScannerWrapper::getCurrentPulseNumber
            )
        )
        .add_property(
            "numRays",
            static_cast<int(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getNumRays
            ),
            static_cast<void(PyScannerWrapper::*)(int const)>(
                &PyScannerWrapper::setNumRays
            )
        )
        .def(
            "getNumRays",
            static_cast<int(PyScannerWrapper::*)(size_t const) const>(
                &PyScannerWrapper::getNumRays
            )
        )
        .def(
            "setNumRays",
            static_cast<void(PyScannerWrapper::*)(int const, size_t const)>(
                &PyScannerWrapper::setNumRays
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "pulseLength_ns",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getPulseLength_ns
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setPulseLength_ns
            )
        )
        .def(
            "getPulseLength_ns",
            static_cast<double(PyScannerWrapper::*)(size_t const)>(
                &PyScannerWrapper::getPulseLength_ns
            )
        )
        .def(
            "setPulseLength_ns",
            static_cast<
                void(PyScannerWrapper::*)(double const, size_t const)
            >(
                &PyScannerWrapper::setPulseLength_ns
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "beamDivergence",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getBeamDivergence
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setBeamDivergence
            )
        )
        .def(
            "getBeamDivergence",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getBeamDivergence
            )
        )
        .def(
            "setBeamDivergence",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setBeamDivergence
            )
        )
        .def(
            "getLastPulseWasHit",
            static_cast<bool(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getLastPulseWasHit
            )
        )
        .def(
            "setLastPulseWasHit",
            static_cast<void(PyScannerWrapper::*)(bool const, size_t const)>(
                &PyScannerWrapper::setLastPulseWasHit
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "averagePower",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getAveragePower
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setAveragePower
            )
        )
        .def(
            "getAveragePower",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getAveragePower
            )
        )
        .def(
            "setAveragePower",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setAveragePower
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "beamQuality",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getBeamQuality
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setBeamQuality
            )
        )
        .def(
            "getBeamQuality",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getBeamQuality
            )
        )
        .def(
            "setBeamQuality",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setBeamQuality
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "efficiency",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getEfficiency
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setEfficiency
            )
        )
        .def(
            "getEfficiency",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getEfficiency
            )
        )
        .def(
            "setEfficiency",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setEfficiency
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "receiverDiameter",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getReceiverDiameter
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setReceiverDiameter
            )
        )
        .def(
            "getReceiverDiameter",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getReceiverDiameter
            )
        )
        .def(
            "setReceiverDiameter",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setReceiverDiameter
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "visibility",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getVisibility
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setVisibility
            )
        )
        .def(
            "getVisibility",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getVisibility
            )
        )
        .def(
            "setVisibility",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setVisibility
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "wavelength",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getWavelength
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setWavelength
            )
        )
        .def(
            "getWavelength",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getWavelength
            )
        )
        .def(
            "setWavelength",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setWavelength
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "atmosphericExtinction",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getAtmosphericExtinction
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setAtmosphericExtinction
            )
        )
        .def(
            "getAtmosphericExtinction",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getAtmosphericExtinction
            )
        )
        .def(
            "setAtmosphericExtinction",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setAtmosphericExtinction
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "beamWaistRadius",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getBeamWaistRadius
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setBeamWaistRadius
            )
        )
        .def(
            "getBeamWaistRadius",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getBeamWaistRadius
            )
        )
        .def(
            "setBeamWaistRadius",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setBeamWaistRadius
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "maxNOR",
            static_cast<int(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getMaxNOR
            ),
            static_cast<void(PyScannerWrapper::*)(int const)>(
                &PyScannerWrapper::setMaxNOR
            )
        )
        .def(
            "getMaxNOR",
            static_cast<int(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getMaxNOR
            )
        )
        .def(
            "setMaxNOR",
            static_cast<void(PyScannerWrapper::*)(int const, size_t const)>(
                &PyScannerWrapper::setMaxNOR
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "bt2",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getBt2
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setBt2
            )
        )
        .def(
            "getBt2",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getBt2
            )
        )
        .def(
            "setBt2",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setBt2
            )
        )
        .add_property( // Only access first device. Use get/set for n device
            "dr2",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getDr2
            ),
            static_cast<void(PyScannerWrapper::*)(double const)>(
                &PyScannerWrapper::setDr2
            )
        )
        .def(
            "getDr2",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getDr2
            )
        )
        .def(
            "setDr2",
            static_cast<void(PyScannerWrapper::*)(double const, size_t const)>(
                &PyScannerWrapper::setDr2
            )
        )
        .add_property(
            "active",
            &PyScannerWrapper::isActive,
            &PyScannerWrapper::setActive
        )
        .add_property(
            "writeWaveform",
            &PyScannerWrapper::isWriteWaveform,
            &PyScannerWrapper::setWriteWaveform
        )
        .add_property(
            "calcEchowidth",
            &PyScannerWrapper::isCalcEchowidth,
            &PyScannerWrapper::setCalcEchowidth
        )
        .add_property(
            "fullWaveNoise",
            &PyScannerWrapper::isFullWaveNoise,
            &PyScannerWrapper::setFullWaveNoise
        )
        .add_property(
            "platformNoiseDisabled",
            &PyScannerWrapper::isPlatformNoiseDisabled,
            &PyScannerWrapper::setPlatformNoiseDisabled
        )
        .def(
            "getSupportedPulseFrequencies",
            static_cast<PyIntegerList*(PyScannerWrapper::*)()>(
                &PyScannerWrapper::getSupportedPulseFrequencies
            ),
            return_internal_reference<>()
        )
        .def(
            "getSupportedPulseFrequencies",
            static_cast<PyIntegerList*(PyScannerWrapper::*)(size_t const)>(
                &PyScannerWrapper::getSupportedPulseFrequencies
            ),
            return_internal_reference<>()
        )
        .def(
            "getRelativeAttitude",
            static_cast<Rotation &(PyScannerWrapper::*)()>(
                &PyScannerWrapper::getRelativeAttitudeByReference
            ),
            return_internal_reference<>()
        )
        .def(
            "getRelativeAttitude",
            static_cast<Rotation &(PyScannerWrapper::*)(size_t const)>(
                &PyScannerWrapper::getRelativeAttitudeByReference
            ),
            return_internal_reference<>()
        )
        .def(
            "getRelativePosition",
            static_cast<PythonDVec3 *(PyScannerWrapper::*)()>(
                &PyScannerWrapper::getRelativePosition
            ),
            return_value_policy<manage_new_object>()
        )
        .def(
            "getRelativePosition",
            static_cast<PythonDVec3 *(PyScannerWrapper::*)(size_t const)>(
                &PyScannerWrapper::getRelativePosition
            ),
            return_value_policy<manage_new_object>()
        )
        .def(
            "getIntersectionHandlingNoiseSource",
            &PyScannerWrapper::getIntersectionHandlingNoiseSource,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getRandGen1",
            &PyScannerWrapper::getRandGen1,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getRandGen2",
            &PyScannerWrapper::getRandGen2,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getScannerHead",
            static_cast<ScannerHead&(PyScannerWrapper::*)()>(
                &PyScannerWrapper::getScannerHead
            ),
            return_internal_reference<>()
        )
        .def(
            "getScannerHead",
            static_cast<ScannerHead&(PyScannerWrapper::*)(size_t const)>(
                &PyScannerWrapper::getScannerHead
            ),
            return_internal_reference<>()
        )
        .def(
            "getBeamDeflector",
            static_cast<
                PyBeamDeflectorWrapper*(PyScannerWrapper::*)()
            >(&PyScannerWrapper::getPyBeamDeflector),
            return_value_policy<manage_new_object>()
        )
        .def(
            "getBeamDeflector",
            static_cast<
                PyBeamDeflectorWrapper*(PyScannerWrapper::*)(size_t const)
                >(&PyScannerWrapper::getPyBeamDeflector),
            return_value_policy<manage_new_object>()
        )
        .def(
            "getDetector",
            static_cast<
                PyDetectorWrapper*(PyScannerWrapper::*)()
            >(&PyScannerWrapper::getPyDetectorWrapper),
            return_value_policy<manage_new_object>()
        )
        .def(
            "getDetector",
            static_cast<
                PyDetectorWrapper*(PyScannerWrapper::*)(size_t const)
            >(&PyScannerWrapper::getPyDetectorWrapper),
            return_value_policy<manage_new_object>()
        )
        .def(
            "calcRaysNumber",
            static_cast<void(PyScannerWrapper::*)()>(
                &PyScannerWrapper::calcRaysNumber
            ),
            return_value_policy<manage_new_object>()
        )
        .def(
            "calcRaysNumber",
            static_cast<void(PyScannerWrapper::*)(size_t const)>(
                &PyScannerWrapper::calcRaysNumber
            ),
            return_value_policy<manage_new_object>()
        )
        .def(
            "calcAtmosphericAttenuation",
            static_cast<double(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::calcAtmosphericAttenuation
            )
        )
        .def(
            "calcAtmosphericAttenuation",
            static_cast<double(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::calcAtmosphericAttenuation
            )
        )
        .add_property(
            "fixedIncidenceAngle",
            &PyScannerWrapper::isFixedIncidenceAngle,
            &PyScannerWrapper::setFixedIncidenceAngle
        )
        .add_property( // It was not in ns before, but in seconds
            "trajectoryTimeInterval",
            &PyScannerWrapper::getTrajectoryTimeInterval,
            &PyScannerWrapper::setTrajectoryTimeInterval
        )
        .add_property(
            "trajectoryTimeInterval_ns",
            &PyScannerWrapper::getTrajectoryTimeInterval,
            &PyScannerWrapper::setTrajectoryTimeInterval
        )
        .add_property(  // Only access first device. Use get/set for n device.
            "deviceId",
            static_cast<std::string(PyScannerWrapper::*)()const>(
                &PyScannerWrapper::getDeviceId
            ),
            static_cast<void(PyScannerWrapper::*)(std::string const)>(
                &PyScannerWrapper::setDeviceId
            )
        )
        .def(
            "getDeviceId",
            static_cast<std::string(PyScannerWrapper::*)(size_t const)const>(
                &PyScannerWrapper::getDeviceId
            )
        )
        .def(
            "setDeviceId",
            static_cast<
                void(PyScannerWrapper::*)(std::string const, size_t const)
            >(&PyScannerWrapper::setDeviceId)
        )
        .add_property(
            "id",
            &PyScannerWrapper::getScannerId,
            &PyScannerWrapper::setScannerId
        )
        .def("getNumDevices", &PyScannerWrapper::getNumDevices)
    ;

    // Register FWFSettings
    class_<FWFSettings>("FWFSettings", no_init)
        .add_property(
            "binSize_ns",
            &FWFSettings::binSize_ns,
            &FWFSettings::binSize_ns
        )
        .add_property(
            "minEchoWidth",
            &FWFSettings::minEchoWidth,
            &FWFSettings::minEchoWidth
        )
        .add_property(
            "peakEnergy",
            &FWFSettings::peakEnergy,
            &FWFSettings::peakEnergy
        )
        .add_property(
            "apertureDiameter",
            &FWFSettings::apertureDiameter,
            &FWFSettings::apertureDiameter
        )
        .add_property(
            "scannerEfficiency",
            &FWFSettings::scannerEfficiency,
            &FWFSettings::scannerEfficiency
        )
        .add_property(
            "atmosphericVisiblity",
            &FWFSettings::atmosphericVisibility,
            &FWFSettings::atmosphericVisibility
        )
        .add_property(
            "scannerWaveLength",
            &FWFSettings::scannerWaveLength,
            &FWFSettings::scannerWaveLength
        )
        .add_property(
            "beamDivergence",
            &FWFSettings::beamDivergence_rad,
            &FWFSettings::beamDivergence_rad
        )
        .add_property(
            "pulseLength_ns",
            &FWFSettings::pulseLength_ns,
            &FWFSettings::pulseLength_ns
        )
        .add_property(
            "beamSampleQuality",
            &FWFSettings::beamSampleQuality,
            &FWFSettings::beamSampleQuality
        )
        .add_property(
            "winSize_ns",
            &FWFSettings::winSize_ns,
            &FWFSettings::winSize_ns
        )
        .add_property(
            "maxFullwaveRange_ns",
            &FWFSettings::maxFullwaveRange_ns,
            &FWFSettings::maxFullwaveRange_ns
        )
        .def("toString", &FWFSettings::toString)
    ;

    // Register DVec3 (glm::dvec3 wrapper)
    class_<PythonDVec3>("DVec3", no_init)
        .add_property("x", &PythonDVec3::getX, &PythonDVec3::setX)
        .add_property("y", &PythonDVec3::getY, &PythonDVec3::setY)
        .add_property("z", &PythonDVec3::getZ, &PythonDVec3::setZ)
    ;

    // Register Rotation
    class_<Rotation>("Rotation", no_init)
        .add_property("q0", &Rotation::getQ0, &Rotation::setQ0)
        .add_property("q1", &Rotation::getQ1, &Rotation::setQ1)
        .add_property("q2", &Rotation::getQ2, &Rotation::setQ2)
        .add_property("q3", &Rotation::getQ3, &Rotation::setQ3)
        .def(
            "getAxis",
            // The unary operator+ is a C++ trick to convert a capture-less
            // lambda to a function pointer, so that we can do this inline.
            +[](Rotation &r) {
                return new PythonDVec3(r.getAxis());
            },
            //&Rotation::getAxisPython,
            return_value_policy<manage_new_object>()
        )
        .def("getAngle", &Rotation::getAngle)
    ;

    // Register ScannerHead
    class_<ScannerHead>("ScannerHead", no_init)
        .add_property(
            "rotatePerSecMax",
            &ScannerHead::getRotatePerSecMax,
            &ScannerHead::setRotatePerSecMax
        )
        .add_property(
            "rotatePerSec",
            &ScannerHead::getRotatePerSec_rad,
            &ScannerHead::setRotatePerSec_rad
        )
        .add_property(
            "rotateStop",
            &ScannerHead::getRotateStop,
            &ScannerHead::setRotateStop
        )
        .add_property(
            "rotateStart",
            &ScannerHead::getRotateStart,
            &ScannerHead::setRotateStart
        )
        .add_property(
            "rotateRange",
            &ScannerHead::getRotateRange,
            &ScannerHead::setRotateRange
        )
        .add_property(
            "currentRotateAngle",
            &ScannerHead::getRotateCurrent,
            &ScannerHead::setCurrentRotateAngle_rad
        )
        .def(
            "getMountRelativeAttitude",
            &ScannerHead::getMountRelativeAttitudeByReference,
            return_internal_reference<>()
        )
    ;

    // Register AbstractBeamDeflector
    class_<PyBeamDeflectorWrapper>("AbstractBeamDeflector", no_init)
        .add_property(
            "scanFreqMax",
            &PyBeamDeflectorWrapper::getScanFreqMax,
            &PyBeamDeflectorWrapper::setScanFreqMax
        )
        .add_property(
            "scanFreqMin",
            &PyBeamDeflectorWrapper::getScanFreqMin,
            &PyBeamDeflectorWrapper::setScanFreqMin
        )
        .add_property(
            "scanAngleMax",
            &PyBeamDeflectorWrapper::getScanAngleMax,
            &PyBeamDeflectorWrapper::setScanAngleMax
        )
        .add_property(
            "scanFreq",
            &PyBeamDeflectorWrapper::getScanFreq,
            &PyBeamDeflectorWrapper::setScanFreq
        )
        .add_property(
            "scanAngle",
            &PyBeamDeflectorWrapper::getScanAngle,
            &PyBeamDeflectorWrapper::setScanAngle
        )
        .add_property(
            "verticalAngleMin",
            &PyBeamDeflectorWrapper::getVerticalAngleMin,
            &PyBeamDeflectorWrapper::setVerticalAngleMin
        )
        .add_property(
            "verticalAngleMax",
            &PyBeamDeflectorWrapper::getVerticalAngleMax,
            &PyBeamDeflectorWrapper::setVerticalAngleMax
        )
        .add_property(
            "currentBeamAngle",
            &PyBeamDeflectorWrapper::getCurrentBeamAngle,
            &PyBeamDeflectorWrapper::setCurrentBeamAngle
        )
        .add_property(
            "angleDiff",
            &PyBeamDeflectorWrapper::getAngleDiff,
            &PyBeamDeflectorWrapper::setAngleDiff
        )
        .add_property(
            "cachedAngleBetweenPulses",
            &PyBeamDeflectorWrapper::getCachedAngleBetweenPulses,
            &PyBeamDeflectorWrapper::setCachedAngleBetweenPulses
        )
        .def(
            "getEmitterRelativeAttitude",
            &PyBeamDeflectorWrapper::getEmitterRelativeAttitude,
            return_internal_reference<>()
        )
        .def(
            "getOpticsType",
            static_cast<string(PyBeamDeflectorWrapper::*)()const>(
                &PyBeamDeflectorWrapper::getOpticsType
            )
        )
        .def(
            "getOpticsType",
            static_cast<string(PyBeamDeflectorWrapper::*)(size_t const)const>(
                &PyBeamDeflectorWrapper::getOpticsType
            )
        )
    ;

    // Register AbstractDetector
    class_<PyDetectorWrapper>("AbstractDetector", no_init)
        .add_property(
            "accuracy",
            &PyDetectorWrapper::getAccuracy,
            &PyDetectorWrapper::setAccuracy
        )
        .add_property(
            "rangeMin",
            &PyDetectorWrapper::getRangeMin,
            &PyDetectorWrapper::setRangeMin
        )
        .add_property(
            "rangeMax",
            &PyDetectorWrapper::getRangeMax,
            &PyDetectorWrapper::setRangeMax
        )
        .add_property(
            "lasScale",
            &PyDetectorWrapper::getLasScale,
            &PyDetectorWrapper::setLasScale
        )
    ;

    // Register list<int>
    class_<PyIntegerList>("IntegerList", no_init)
        .def(
            "__getitem__",
            &PyIntegerList::get
        )
        .def(
            "__setitem__",
            &PyIntegerList::set
        )
        .def(
            "__len__",
            &PyIntegerList::length
        )
        .def(
            "get",
            &PyIntegerList::get
        )
        .def(
            "set",
            &PyIntegerList::set
        )
        .def(
            "insert",
            &PyIntegerList::insert
        )
        .def(
            "erase",
            &PyIntegerList::erase
        )
        .def(
            "length",
            &PyIntegerList::length
        )
    ;

    // Register vector<double>
    class_<PyDoubleVector>("DoubleVector", no_init)
        .def(
            "__getitem__",
            &PyDoubleVector::get
        )
        .def(
            "__setitem__",
            &PyDoubleVector::set
        )
        .def(
            "__len__",
            &PyDoubleVector::length
        )
        .def(
            "get",
            &PyDoubleVector::get
        )
        .def(
            "set",
            &PyDoubleVector::set
        )
        .def(
            "insert",
            &PyDoubleVector::insert
        )
        .def(
            "erase",
            &PyDoubleVector::erase
        )
        .def(
            "length",
            &PyDoubleVector::length
        )
    ;

    // Register vector<string>
    class_<PyStringVector>("StringVector", no_init)
        .def(
            "__getitem__",
            &PyStringVector::get
        )
        .def(
            "__setitem__",
            &PyStringVector::set
        )
        .def(
            "__len__",
            &PyStringVector::length
        )
        .def(
            "get",
            &PyStringVector::get
        )
        .def(
            "set",
            &PyStringVector::set
        )
        .def(
            "insert",
            &PyStringVector::insert
        )
        .def(
            "erase",
            &PyStringVector::erase
        )
        .def(
            "length",
            &PyStringVector::length
        )
    ;

    // Register NoiseSource
    class_<PyNoiseSourceWrapper>("NoiseSource", no_init)
        .add_property(
            "clipMin",
            &PyNoiseSourceWrapper::getClipMin,
            &PyNoiseSourceWrapper::setClipMin
        )
        .add_property(
            "clipMax",
            &PyNoiseSourceWrapper::getClipMax,
            &PyNoiseSourceWrapper::setClipMax
        )
        .add_property(
            "enabled",
            &PyNoiseSourceWrapper::isEnabled,
            &PyNoiseSourceWrapper::setEnabled
        )
        .def(
            "isFixedValueEnabled",
            &PyNoiseSourceWrapper::isFixedValueEnabled
        )
        .add_property(
            "fixedLifespan",
            &PyNoiseSourceWrapper::getFixedLifespan,
            &PyNoiseSourceWrapper::setFixedLifespan
        )
        .add_property(
            "fixedValueRemainingUses",
            &PyNoiseSourceWrapper::getFixedValueRemainingUses,
            &PyNoiseSourceWrapper::setFixedValueRemainingUses
        )
    ;

    // Register RandomnessGenerator
    class_<PyRandomnessGeneratorWrapper>("RandomnessGenerator", no_init)
        .def(
            "computeUniformRealDistribution",
            &PyRandomnessGeneratorWrapper::computeUniformRealDistribution
        )
        .def(
            "uniformRealDistributionNext",
            &PyRandomnessGeneratorWrapper::uniformRealDistributionNext
        )
        .def(
            "computeNormalDistribution",
            &PyRandomnessGeneratorWrapper::computeNormalDistribution
        )
        .def(
            "normalDistributionNext",
            &PyRandomnessGeneratorWrapper::normalDistributionNext
        )
    ;

    // Register Platform
    class_<PyPlatformWrapper>("Platform", no_init)
        .add_property(
            "lastCheckZ",
            &PyPlatformWrapper::getLastCheckZ,
            &PyPlatformWrapper::setLastCheckZ
        )
        .add_property(
            "dmax",
            &PyPlatformWrapper::getDmax,
            &PyPlatformWrapper::setDmax
        )
        .add_property(
            "movePerSec",
            &PyPlatformWrapper::getMovePerSec,
            &PyPlatformWrapper::setMovePerSec
        )
        .add_property(
            "onGround",
            &PyPlatformWrapper::isOnGround,
            &PyPlatformWrapper::setOnGround
        )
        .add_property(
            "stopAndTurn",
            &PyPlatformWrapper::isStopAndTurn,
            &PyPlatformWrapper::setStopAndTurn
        )
        .add_property(
            "slowdownEnabled",
            &PyPlatformWrapper::isSlowdownEnabled,
            &PyPlatformWrapper::setSlowdownEnabled
        )
        /*.add_property(
            "yawAtDeparture",
            &PyPlatformWrapper::getYawAtDeparture,
            &PyPlatformWrapper::setYawAtDeparture
        )*/
        .add_property(
            "smoothTurn",
            &PyPlatformWrapper::isSmoothTurn,
            &PyPlatformWrapper::setSmoothTurn
        )
        .add_property(
            "mSetOrientationOnLegInit",
            &PyPlatformWrapper::isOrientationOnLegInit,
            &PyPlatformWrapper::setOrientationOnLegInit
        )
        .def(
            "getPositionXNoiseSource",
            &PyPlatformWrapper::getPositionXNoiseSource,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getPositionYNoiseSource",
            &PyPlatformWrapper::getPositionYNoiseSource,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getPositionZNoiseSource",
            &PyPlatformWrapper::getPositionZNoiseSource,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getAttitudeXNoiseSource",
            &PyPlatformWrapper::getAttitudeXNoiseSource,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getAttitudeYNoiseSource",
            &PyPlatformWrapper::getAttitudeYNoiseSource,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getAttitudeZNoiseSource",
            &PyPlatformWrapper::getAttitudeZNoiseSource,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getRelativePosition",
            &PyPlatformWrapper::getRelativePosition,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getRelativeAttitude",
            &PyPlatformWrapper::getRelativeAttitude,
            return_internal_reference<>()
        )
        .def(
            "getLastGroundCheck",
            &PyPlatformWrapper::getLastGroundCheck,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getNextWaypointPosition",
            &PyPlatformWrapper::getNextWaypointPosition,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getPositionPython",
            &PyPlatformWrapper::getPositionPython,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getAttitudePython",
            &PyPlatformWrapper::getAttitudePython,
            return_internal_reference<>()
        )
        .def(
            "getCachedAbsolutePosition",
            &PyPlatformWrapper::getCachedAbsolutePosition,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getCachedAbsoluteAttitude",
            &PyPlatformWrapper::getCachedAbsoluteAttitude,
            return_internal_reference<>()
        )
        .def(
            "getCachedCurrentDir",
            &PyPlatformWrapper::getCachedCurrentDir,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getCachedCurrentDirXY",
            &PyPlatformWrapper::getCachedCurrentDirXY,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getCachedVectorToTarget",
            &PyPlatformWrapper::getCachedVectorToTarget,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getCachedVectorToTargetXY",
            &PyPlatformWrapper::getCachedVectorToTargetXY,
            return_value_policy<manage_new_object>()
        )
    ;

    // Register Primitive
    class_<PyPrimitiveWrapper>("Primitive", no_init)
        .def(
            "getScenePart",
            &PyPrimitiveWrapper::getScenePart,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getMaterial",
            &PyPrimitiveWrapper::getMaterial,
            return_internal_reference<>()
        )
        .def(
            "getAABB",
            &PyPrimitiveWrapper::getAABB,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getCentroid",
            &PyPrimitiveWrapper::getCentroid,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getIncidenceAngle",
            &PyPrimitiveWrapper::getIncidenceAngle
        )
        .def(
            "getRayIntersection",
            &PyPrimitiveWrapper::getRayIntersection,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getRayIntersectionDistance",
            &PyPrimitiveWrapper::getRayIntersectionDistance
        )
        .def(
            "getNumVertices",
            &PyPrimitiveWrapper::getNumVertices
        )
        .def(
            "getVertex",
            &PyPrimitiveWrapper::getVertex,
            return_value_policy<manage_new_object>()
        )
        .def(
            "update",
            &PyPrimitiveWrapper::update
        )
    ;

    // Register Material
    class_<Material>("Material", no_init)
        .add_property(
            "name",
            &Material::name,
            &Material::name
        )
        .add_property(
            "isGround",
            &Material::isGround,
            &Material::isGround
        )
        .add_property(
            "useVertexColors",
            &Material::useVertexColors,
            &Material::useVertexColors
        )
        .add_property(
            "matFilePath",
            &Material::matFilePath,
            &Material::matFilePath
        )
        .add_property(
            "map_Kd",
            &Material::map_Kd,
            &Material::map_Kd
        )
        .add_property(
            "reflectance",
            &Material::reflectance,
            &Material::reflectance
        )
        .add_property(
            "specularity",
            &Material::specularity,
            &Material::specularity
        )
        .add_property(
            "specularExponent",
            &Material::specularExponent,
            &Material::specularExponent
        )
        .add_property(
            "classification",
            &Material::classification,
            &Material::classification
        )
        .add_property(
            "spectra",
            &Material::spectra,
            &Material::spectra
        )
        .add_property("ka0",
            +[](Material &m){ return m.ka[0]; },
            +[](Material &m, double v){ m.ka[0] = v; }
        )
        .add_property("ka1",
            +[](Material &m){ return m.ka[1]; },
            +[](Material &m, double v){ m.ka[1] = v; }
        )
        .add_property("ka2",
            +[](Material &m){ return m.ka[2]; },
            +[](Material &m, double v){ m.ka[2] = v; }
        )
        .add_property("ka3",
            +[](Material &m){ return m.ka[3]; },
            +[](Material &m, double v){ m.ka[3] = v; }
        )
        .add_property("kd0",
            +[](Material &m){ return m.kd[0]; },
            +[](Material &m, double v){ m.kd[0] = v; }
        )
        .add_property("kd1",
            +[](Material &m){ return m.kd[1]; },
            +[](Material &m, double v){ m.kd[1] = v; }
        )
        .add_property("kd2",
            +[](Material &m){ return m.kd[2]; },
            +[](Material &m, double v){ m.kd[2] = v; }
        )
        .add_property("kd3",
            +[](Material &m){ return m.kd[3]; },
            +[](Material &m, double v){ m.kd[3] = v; }
        )
        .add_property("ks0",
            +[](Material &m){ return m.ks[0]; },
            +[](Material &m, double v){ m.ks[0] = v; }
        )
        .add_property("ks1",
            +[](Material &m){ return m.ks[1]; },
            +[](Material &m, double v){ m.ks[1] = v; }
        )
        .add_property("ks2",
            +[](Material &m){ return m.ks[2]; },
            +[](Material &m, double v){ m.ks[2] = v; }
        )
        .add_property("ks3",
            +[](Material &m){ return m.ks[3]; },
            +[](Material &m, double v){ m.ks[3] = v; }
        );

    // Register ScenePart
    class_<PyScenePartWrapper>("ScenePart", no_init)
        .add_property(
            "id",
            &PyScenePartWrapper::getId,
            &PyScenePartWrapper::setId
        )
        .def(
            "getOrigin",
            &PyScenePartWrapper::getOrigin,
            return_value_policy<manage_new_object>()
        )
        .def(
            "setOrigin",
            &PyScenePartWrapper::setOrigin
        )
        .def(
            "setRotation",
            &PyScenePartWrapper::setRotation
        )
        .def(
            "getRotation",
            &PyScenePartWrapper::getRotation,
            return_internal_reference<>()
        )
        .add_property(
            "scale",
            &PyScenePartWrapper::getScale,
            &PyScenePartWrapper::setScale
        )
        .def(
            "isDynamicMovingObject",
            &PyScenePartWrapper::isDynamicMovingObject
        )
        .add_property(
            "dynObjectStep",
            &PyScenePartWrapper::getDynObjectStep,
            &PyScenePartWrapper::setDynObjectStep
        )
        .add_property(
            "observerStep",
            &PyScenePartWrapper::getObserverStep,
            &PyScenePartWrapper::setObserverStep
        )
    ;

    // Register Scene
    class_<PySceneWrapper>("Scene", no_init)
        .def(
            "newTriangle",
            &PySceneWrapper::newTriangle,
            return_value_policy<manage_new_object>()
        )
        .def(
            "newDetailedVoxel",
            &PySceneWrapper::newDetailedVoxel,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getPrimitive",
            &PySceneWrapper::getPrimitive,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getAABB",
            &PySceneWrapper::getAABB,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getGroundPointAt",
            &PySceneWrapper::getGroundPointAt,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getIntersection",
            &PySceneWrapper::getIntersection,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getShift",
            &PySceneWrapper::getShift,
            return_value_policy<manage_new_object>()
        )
        .def(
            "finalizeLoading",
            &PySceneWrapper::finalizeLoading
        )
        .def(
            "writeObject",
            &PySceneWrapper::writeObject
        )
        .def(
            "getNumSceneParts",
            &PySceneWrapper::getNumSceneParts
        )
        .def(
            "getScenePart",
            &PySceneWrapper::getScenePart,
            return_value_policy<manage_new_object>()
        )
        .add_property(
            "dynSceneStep",
            &PySceneWrapper::getDynSceneStep,
            &PySceneWrapper::setDynSceneStep
        )
    ;

    // Register AABB
    class_<PyAABBWrapper>("AABB", no_init)
        .def(
            "getMinVertex",
            &PyAABBWrapper::getMinVertex,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getMaxVertex",
            &PyAABBWrapper::getMaxVertex,
            return_value_policy<manage_new_object>()
        )
        .def(
            "toString",
            &PyAABBWrapper::toString
        )
    ;

    // Register Vertex
    class_<PyVertexWrapper>("Vertex", no_init)
        .def(
            "getPosition",
            &PyVertexWrapper::getPosition,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getNormal",
            &PyVertexWrapper::getNormal,
            return_value_policy<manage_new_object>()
        )
    ;

    // Register Triangle
    class_<PyTriangleWrapper, bases<PyPrimitiveWrapper>>
    ("Triangle", no_init)
        .def(
            "getFaceNormal",
            &PyTriangleWrapper::getFaceNormal,
            return_value_policy<manage_new_object>()
        )
        .def(
            "toString",
            &PyTriangleWrapper::toString
        )
    ;

    // Register DetailedVoxel
    class_<PyDetailedVoxelWrapper, bases<PyPrimitiveWrapper>>
    ("DetailedVoxel", no_init)
        .add_property(
            "nbEchos",
            &PyDetailedVoxelWrapper::getNbEchos,
            &PyDetailedVoxelWrapper::setNbEchos
        )
        .add_property(
            "nbSampling",
            &PyDetailedVoxelWrapper::getNbSampling,
            &PyDetailedVoxelWrapper::setNbSampling
        )
        .def(
            "getNumberOfDoubleValues",
            &PyDetailedVoxelWrapper::getNumberOfDoubleValues
        )
        .def(
            "getDoubleValue",
            &PyDetailedVoxelWrapper::getDoubleValue
        )
        .def(
            "setDoubleValue",
            &PyDetailedVoxelWrapper::setDoubleValue
        )
        .add_property(
            "maxPad",
            &PyDetailedVoxelWrapper::getMaxPad,
            &PyDetailedVoxelWrapper::setMaxPad
        )
    ;

    // Register MeasurementVector
    class_<PyMeasurementVectorWrapper>("MeasurementVector", no_init)
        .def(
            "__getitem__",
            &PyMeasurementVectorWrapper::get,
            return_value_policy<manage_new_object>()
        )
        .def("__len__",
            &PyMeasurementVectorWrapper::length
        )
        .def(
            "get",
            &PyMeasurementVectorWrapper::get,
            return_value_policy<manage_new_object>()
        )
        .def(
            "erase",
            &PyMeasurementVectorWrapper::erase
        )
        .def(
            "length",
            &PyMeasurementVectorWrapper::length
        )
    ;

    // Register Measurement
    class_<PyMeasurementWrapper>("Measurement", no_init)
        .add_property(
            "hitObjectId",
            &PyMeasurementWrapper::getHitObjectId,
            &PyMeasurementWrapper::setHitObjectId
        )
        .def(
            "getPosition",
            &PyMeasurementWrapper::getPosition,
            return_value_policy<manage_new_object>()
        )
        .def(
            "setPosition",
            &PyMeasurementWrapper::setPosition
        )
        .def(
            "getBeamDirection",
            &PyMeasurementWrapper::getBeamDirection,
            return_value_policy<manage_new_object>()
        )
        .def(
            "setBeamDirection",
            &PyMeasurementWrapper::setBeamDirection
        )
        .def(
            "getBeamOrigin",
            &PyMeasurementWrapper::getBeamOrigin,
            return_value_policy<manage_new_object>()
        )
        .def(
            "setBeamOrigin",
            &PyMeasurementWrapper::setBeamOrigin
        )
        .add_property(
            "distance",
            &PyMeasurementWrapper::getDistance,
            &PyMeasurementWrapper::setDistance
        )
        .add_property(
            "intensity",
            &PyMeasurementWrapper::getIntensity,
            &PyMeasurementWrapper::setIntensity
        )
        .add_property(
            "echoWidth",
            &PyMeasurementWrapper::getEchoWidth,
            &PyMeasurementWrapper::setEchoWidth
        )
        .add_property(
            "returnNumber",
            &PyMeasurementWrapper::getReturnNumber,
            &PyMeasurementWrapper::setReturnNumber
        )
        .add_property(
            "pulseReturnNumber",
            &PyMeasurementWrapper::getPulseReturnNumber,
            &PyMeasurementWrapper::setPulseReturnNumber
        )
        .add_property(
            "fullwaveIndex",
            &PyMeasurementWrapper::getFullwaveIndex,
            &PyMeasurementWrapper::setFullwaveIndex
        )
        .add_property(
            "classification",
            &PyMeasurementWrapper::getClassification,
            &PyMeasurementWrapper::setClassification
        )
        .add_property(
            "gpsTime",
            &PyMeasurementWrapper::getGpsTime,
            &PyMeasurementWrapper::setGpsTime
        )
    ;

    // Register TrajectoryVector
    class_<PyTrajectoryVectorWrapper>("TrajectoryVector", no_init)
        .def(
            "__getitem__",
            &PyTrajectoryVectorWrapper::get,
            return_value_policy<manage_new_object>()
        )
        .def(
            "__len__",
            &PyTrajectoryVectorWrapper::length
        )
        .def(
            "get",
            &PyTrajectoryVectorWrapper::get,
            return_value_policy<manage_new_object>()
        )
        .def(
            "erase",
            &PyTrajectoryVectorWrapper::erase
        )
        .def(
            "length",
            &PyTrajectoryVectorWrapper::length
        )
    ;

    // Register Trajectory
    class_<PyTrajectoryWrapper>("Trajectory", no_init)
        .add_property(
            "gpsTime",
            &PyTrajectoryWrapper::getGpsTime,
            &PyTrajectoryWrapper::setGpsTime
        )
        .def(
            "getPosition",
            &PyTrajectoryWrapper::getPosition,
            return_value_policy<manage_new_object>()
        )
        .def(
            "setPosition",
            &PyTrajectoryWrapper::setPosition,
            return_value_policy<manage_new_object>()
        )
        .add_property(
            "roll",
            &PyTrajectoryWrapper::getRoll,
            &PyTrajectoryWrapper::setRoll
        )
        .add_property(
            "pitch",
            &PyTrajectoryWrapper::getPitch,
            &PyTrajectoryWrapper::setPitch
        )
        .add_property(
            "yaw",
            &PyTrajectoryWrapper::getYaw,
            &PyTrajectoryWrapper::setYaw
        )
    ;

    // Register PyHeliosOutputWrapper
    class_<PyHeliosOutputWrapper>("HeliosOutput", no_init)
        .add_property(
            "measurements",
            &PyHeliosOutputWrapper::measurements,
            &PyHeliosOutputWrapper::measurements
        )
        .add_property(
            "trajectories",
            &PyHeliosOutputWrapper::trajectories,
            &PyHeliosOutputWrapper::trajectories
        )
        .add_property(
            "finished",
            &PyHeliosOutputWrapper::finished,
            &PyHeliosOutputWrapper::finished
        )
        .add_property(
            "outpath",
            &PyHeliosOutputWrapper::outpath,
            &PyHeliosOutputWrapper::outpath
        )
        .add_property(
            "filepath",
            &PyHeliosOutputWrapper::outpath,
            &PyHeliosOutputWrapper::outpath
        )
        .add_property(
            "outpaths",
            &PyHeliosOutputWrapper::outpaths,
            &PyHeliosOutputWrapper::outpaths
        )
        .add_property(
            "filepaths",
            &PyHeliosOutputWrapper::outpaths,
            &PyHeliosOutputWrapper::outpaths
        )
    ;

    // Register PySimulationCycleCallback
    class_<PySimulationCycleCallback>(
        "SimulationCycleCallback",
        init<PyObject *>()
    )
        .def("call", &PySimulationCycleCallback::operator())
    ;
}
