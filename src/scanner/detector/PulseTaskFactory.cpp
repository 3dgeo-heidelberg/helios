#include <PulseTaskFactory.h>
#include <FullWaveformPulseRunnable.h>
#include <DynFullWaveformPulseRunnable.h>

using std::dynamic_pointer_cast;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
PulseTaskFactory::PulseTaskFactory(Scene &scene) :
    scene(scene)
{
    configureBuildMethod();
}

// ***  FACTORY METHODS  *** //
// ************************* //
shared_ptr<PulseTask> PulseTaskFactory::buildFullWaveformPulseRunnable(
    ScanningPulseProcess const &spp,
    unsigned int const legIndex,
    glm::dvec3 &absoluteBeamOrigin,
    Rotation &absoluteBeamAttitude,
    double const currentGpsTime
) const {
    return make_shared<FullWaveformPulseRunnable>(
        dynamic_pointer_cast<FullWaveformPulseDetector>(spp.getDetector()),
        absoluteBeamOrigin,
        absoluteBeamAttitude,
        spp.getCurrentPulseNumber(),
        currentGpsTime,
        spp.isWriteWaveform(),
        spp.isCalcEchowidth(),
        spp.getAllMeasurements().get(),
        spp.getAllMeasurementsMutex().get(),
        spp.getCycleMeasurements().get(),
        spp.getCycleMeasurementsMutex().get(),
        legIndex
    );
}

shared_ptr<PulseTask> PulseTaskFactory::buildDynFullWaveformPulseRunnable(
    ScanningPulseProcess const &spp,
    unsigned int const legIndex,
    glm::dvec3 &absoluteBeamOrigin,
    Rotation &absoluteBeamAttitude,
    double const currentGpsTime
) const {
    return make_shared<DynFullWaveformPulseRunnable>(
        spp.getDetector()->scanner->platform->scene->getRaycaster()\
            ->makeTemporalClone(),
        dynamic_pointer_cast<FullWaveformPulseDetector>(spp.getDetector()),
        absoluteBeamOrigin,
        absoluteBeamAttitude,
        spp.getCurrentPulseNumber(),
        currentGpsTime,
        spp.isWriteWaveform(),
        spp.isCalcEchowidth(),
        spp.getAllMeasurements().get(),
        spp.getAllMeasurementsMutex().get(),
        spp.getCycleMeasurements().get(),
        spp.getCycleMeasurementsMutex().get(),
        legIndex
    );
}

void PulseTaskFactory::configureBuildMethod(){
    if(scene.hasMovingObjects()){
        _build = [&] (
            ScanningPulseProcess const &spp,
            unsigned int const legIndex,
            glm::dvec3 &absoluteBeamOrigin,
            Rotation &absoluteBeamAttitude,
            double const currentGpsTime
        ) -> shared_ptr<PulseTask>{
            return buildDynFullWaveformPulseRunnable(
                spp,
                legIndex,
                absoluteBeamOrigin,
                absoluteBeamAttitude,
                currentGpsTime
            );
        };
    }
    else{
        _build = [&] (
            ScanningPulseProcess const &spp,
            unsigned int const legIndex,
            glm::dvec3 &absoluteBeamOrigin,
            Rotation &absoluteBeamAttitude,
            double const currentGpsTime
        ) -> shared_ptr<PulseTask>{
            return buildFullWaveformPulseRunnable(
                spp,
                legIndex,
                absoluteBeamOrigin,
                absoluteBeamAttitude,
                currentGpsTime
            );
        };
    }
}
