#ifdef PYTHON_BINDING

#include <boost/python.hpp>
#include <helios_version.h>
#include <PyHeliosSimulation.h>
#include <PythonDVec3.h>
#include <PyBeamDeflectorWrapper.h>
#include <PyDetectorWrapper.h>
#include <PyIntegerList.h>
#include <PyDoubleVector.h>
#include <PyPlatformWrapper.h>
#include <PyPrimitiveWrapper.h>
#include <PySimulationCycleCallback.h>
#include <Material.h>
#include <gdal_priv.h>

using namespace boost::python;

BOOST_PYTHON_MODULE(pyhelios){
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

    // Register PyHeliosSimulation
    def("getVersion", getHeliosVersion, "Obtain the current helios version");
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
            bool
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
            return_internal_reference<>()
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
        .add_property(
            "simFrequency",
            &PyHeliosSimulation::getSimFrequency,
            &PyHeliosSimulation::setSimFrequency
        )
        .add_property(
            "finalOutput",
            &PyHeliosSimulation::finalOutput,
            &PyHeliosSimulation::finalOutput
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
            "numThreads",
            &PyHeliosSimulation::getNumThreads,
            &PyHeliosSimulation::setNumThreads
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
            "beamSampleQuality",
            &ScannerSettings::beamSampleQuality,
            &ScannerSettings::beamSampleQuality
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
            "pulseLength",
            &ScannerSettings::pulseLength_ns,
            &ScannerSettings::pulseLength_ns
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
    ;

    // Register PlatformSettings
    class_<PlatformSettings>("PlatformSettings", no_init)
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
            "yawAtDeparture",
            &PlatformSettings::yawAtDeparture,
            &PlatformSettings::yawAtDeparture
        )
        .add_property(
            "smoothTurn",
            &PlatformSettings::smoothTurn,
            &PlatformSettings::smoothTurn
        )
    ;

    // Register Leg
    class_<Leg, boost::noncopyable>("Leg", no_init)
        .add_property("length", &Leg::getLength, &Leg::setLength)
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
    ;

    // Register Scanner
    class_<Scanner, boost::noncopyable>("Scanner", no_init)
        .add_property("fwfSettings", &Scanner::FWF_settings)
        .add_property(
            "numTimeBins",
            &Scanner::numTimeBins,
            &Scanner::numTimeBins
        )
        .add_property(
            "peakIntensityIndex",
            &Scanner::peakIntensityIndex,
            &Scanner::peakIntensityIndex
        )
        .def(
            "getTimeWave",
            &Scanner::getTimeWave,
            return_value_policy<manage_new_object>()
        )
        .add_property(
            "pulseFreq_Hz",
            &Scanner::getPulseFreq_Hz,
            &Scanner::setPulseFreq_Hz
        )
        .add_property(
            "lastPulseWasHit",
            &Scanner::lastPulseWasHit,
            &Scanner::setLastPulseWasHit
        )
        .def("toString", &Scanner::toString)
        .add_property("numRays", &Scanner::getNumRays, &Scanner::setNumRays)
        .add_property(
            "pulseLength_ns",
            &Scanner::getPulseLength_ns,
            &Scanner::setPulseLength_ns
        )
        .add_property(
            "beamDivergence",
            &Scanner::getBeamDivergence,
            &Scanner::setBeamDivergence
        )
        .add_property(
            "averagePower",
            &Scanner::getAveragePower,
            &Scanner::setAveragePower
        )
        .add_property(
            "beamQuality",
            &Scanner::getBeamQuality,
            &Scanner::setBeamQuality
        )
        .add_property(
            "efficiency",
            &Scanner::getEfficiency,
            &Scanner::setEfficiency
        )
        .add_property(
            "receiverDiameter",
            &Scanner::getReceiverDiameter,
            &Scanner::setReceiverDiameter
        )
        .add_property(
            "visibility",
            &Scanner::getVisibility,
            &Scanner::setVisibility
        )
        .add_property(
            "wavelength",
            &Scanner::getWavelength,
            &Scanner::setWavelength
        )
        .add_property(
            "atmosphericExtinction",
            &Scanner::getAtmosphericExtinction,
            &Scanner::setAtmosphericExtinction
        )
        .add_property(
            "beamWaistRadius",
            &Scanner::getBeamWaistRadius,
            &Scanner::setBeamWaistRadius
        )
        .add_property("bt2", &Scanner::getBt2, &Scanner::setBt2)
        .add_property("dr2", &Scanner::getDr2, &Scanner::setDr2)
        .add_property("active", &Scanner::isActive, &Scanner::setActive)
        .add_property(
            "writeWaveform",
            &Scanner::isWriteWaveform,
            &Scanner::setWriteWaveform
        )
        .add_property(
            "calcEchowidth",
            &Scanner::isCalcEchowidth,
            &Scanner::setCalcEchowidth
        )
        .add_property(
            "fullWaveNoise",
            &Scanner::isFullWaveNoise,
            &Scanner::setFullWaveNoise
        )
        .add_property(
            "platformNoiseDisabled",
            &Scanner::isPlatformNoiseDisabled,
            &Scanner::setPlatformNoiseDisabled
        )
        .def(
            "getSupportedPulseFrequencies",
            &Scanner::getSupportedPulseFrequencies,
            return_internal_reference<>()
        )
        .def(
            "getRelativeAttitude",
            &Scanner::getRelativeAttitudeByReference,
            return_internal_reference<>()
        )
        .def(
            "getRelativePosition",
            &Scanner::getRelativePosition,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getIntersectionHandlingNoiseSource",
            &Scanner::getIntersectionHandlingNoiseSource,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getRandGen1",
            &Scanner::getRandGen1,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getRandGen2",
            &Scanner::getRandGen2,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getScannerHead",
            &Scanner::getScannerHead,
            return_internal_reference<>()
        )
        .def(
            "getBeamDeflector",
            &Scanner::getPyBeamDeflector,
            return_value_policy<manage_new_object>()
        )
        .def(
            "getDetector",
            &Scanner::getPyDetectorWrapper,
            return_value_policy<manage_new_object>()
        )
        .def("calcRaysNumber", &Scanner::calcRaysNumber)
        .def("calcFootprintArea", &Scanner::calcFootprintArea)
        .def(
            "calcAtmosphericAttenuation",
            &Scanner::calcAtmosphericAttenuation
        )
        .def("calcFootprintRadius", &Scanner::calcFootprintRadius)
        .add_property(
            "fixedIncidenceAngle",
            &Scanner::isFixedIncidenceAngle,
            &Scanner::setFixedIncidenceAngle
        )
        .add_property(
            "trajectoryTimeInterval",
            &Scanner::trajectoryTimeInterval,
            &Scanner::trajectoryTimeInterval
        )
        .add_property(
            "deviceId",
            &Scanner::getDeviceId,
            &Scanner::setDeviceId
        )
    ;

    // Register FWFSettings
    class_<FWFSettings>("FWFSettings", no_init)
        .add_property(
            "binSize_ns",
            &FWFSettings::binSize_ns,
            &FWFSettings::binSize_ns
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
            &Rotation::getAxisPython,
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
        .add_property("ka0", &Material::getKa0, &Material::setKa0)
        .add_property("ka1", &Material::getKa1, &Material::setKa1)
        .add_property("ka2", &Material::getKa2, &Material::setKa2)
        .add_property("ka3", &Material::getKa3, &Material::setKa3)
        .add_property("kd0", &Material::getKd0, &Material::setKd0)
        .add_property("kd1", &Material::getKd1, &Material::setKd1)
        .add_property("kd2", &Material::getKd2, &Material::setKd2)
        .add_property("kd3", &Material::getKd3, &Material::setKd3)
        .add_property("ks0", &Material::getKs0, &Material::setKs0)
        .add_property("ks1", &Material::getKs1, &Material::setKs1)
        .add_property("ks2", &Material::getKs2, &Material::setKs2)
        .add_property("ks3", &Material::getKs3, &Material::setKs3)
    ;

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
    ;

    // Register PySimulationCycleCallback
    class_<PySimulationCycleCallback>(
        "SimulationCycleCallback",
        init<PyObject *>()
    )
        .def("call", &PySimulationCycleCallback::operator())
    ;
}

#endif
