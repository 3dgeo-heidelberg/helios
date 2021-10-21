#include <scanner/WarehouseScanningPulseProcess.h>
#include <FullWaveformPulseDetector.h>
#include <FullWaveformPulseRunnable.h>


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
WarehouseScanningPulseProcess::WarehouseScanningPulseProcess(
    std::shared_ptr<AbstractDetector> &detector,
    int &currentPulseNumber,
    bool &writeWaveform,
    bool &calcEchowidth,
    std::shared_ptr<std::vector<Measurement>> &allMeasurements,
    std::shared_ptr<std::mutex> &allMeasurementsMutex,
    std::shared_ptr<std::vector<Measurement>> &cycleMeasurements,
    std::shared_ptr<std::mutex> &cycleMeasurementsMutex,
    PulseTaskDropper &dropper,
    PulseWarehouseThreadPool &pool,
    RandomnessGenerator<double> &randGen1,
    RandomnessGenerator<double> &randGen2,
    UniformNoiseSource<double> &intersectionHandlingNoiseSource
) :
    ScanningPulseProcess(
        detector,
        currentPulseNumber,
        writeWaveform,
        calcEchowidth,
        allMeasurements,
        allMeasurementsMutex,
        cycleMeasurements,
        cycleMeasurementsMutex
    ),
    dropper(dropper),
    pool(pool),
    randGen1(randGen1),
    randGen2(randGen2),
    intersectionHandlingNoiseSource(intersectionHandlingNoiseSource)
{
    if(pool.getPoolSize() > 0){ // Parallel computation
        handler = [&] (
            unsigned int const legIndex,
            glm::dvec3 &absoluteBeamOrigin,
            Rotation &absoluteBeamAttitude,
            double const currentGpsTime
        ) -> void {
            handlePulseComputationParallel(
                legIndex,
                absoluteBeamOrigin,
                absoluteBeamAttitude,
                currentGpsTime
            );
        };
    }
    else{ // Sequential computation
        handler = [&] (
            unsigned int const legIndex,
            glm::dvec3 &absoluteBeamOrigin,
            Rotation &absoluteBeamAttitude,
            double const currentGpsTime
        ) -> void {
            handlePulseComputationSequential(
                legIndex,
                absoluteBeamOrigin,
                absoluteBeamAttitude,
                currentGpsTime
            );
        };
    }

    // Start thread pool threads
    pool.start();
}

// ***  PULSE COMPUTATION  *** //
// *************************** //
void WarehouseScanningPulseProcess::onLegComplete(){
    // Consume pending dropper, if any
    dropper.drop(
        apMatrix,
        randGen1,
        randGen2,
        intersectionHandlingNoiseSource
    );

    // Assist thread pool with pending tasks (on WarehouseThreadPool::join atm)
    shared_ptr<PulseTaskDropper> task;
    while( (task=pool.get()) != nullptr){
        (*task)(apMatrix, randGen1, randGen2, intersectionHandlingNoiseSource);
    }

    // Wait for threads to finish
    pool.join();
}
void WarehouseScanningPulseProcess::onSimulationFinished(){
    pool.finish();
}

// ***  INNER PULSE COMPUTATION  *** //
// ********************************* //
void WarehouseScanningPulseProcess::handlePulseComputationSequential(
    unsigned int const legIndex,
    glm::dvec3 &absoluteBeamOrigin,
    Rotation &absoluteBeamAttitude,
    double const currentGpsTime
){
    // Sequential pulse computation
    FullWaveformPulseRunnable worker = FullWaveformPulseRunnable(
        std::dynamic_pointer_cast<FullWaveformPulseDetector>(detector),
        absoluteBeamOrigin,
        absoluteBeamAttitude,
        currentPulseNumber,
        currentGpsTime,
        writeWaveform,
        calcEchowidth,
        (allMeasurements == nullptr) ? nullptr : allMeasurements.get(),
        (allMeasurementsMutex == nullptr) ?
            nullptr : allMeasurementsMutex.get(),
        (cycleMeasurements == nullptr) ? nullptr : cycleMeasurements.get(),
        (cycleMeasurementsMutex == nullptr) ?
            nullptr : cycleMeasurementsMutex.get(),
        legIndex
    );
    worker( // call functor
        apMatrix,
        randGen1,
        randGen2,
        intersectionHandlingNoiseSource
    );

}
void WarehouseScanningPulseProcess::handlePulseComputationParallel(
    unsigned int const legIndex,
    glm::dvec3 &absoluteBeamOrigin,
    Rotation &absoluteBeamAttitude,
    double const currentGpsTime
){
    // Submit pulse computation functor to thread pool
    char const status = dropper.tryAdd(
        pool,
        std::make_shared<FullWaveformPulseRunnable>(
            std::dynamic_pointer_cast<FullWaveformPulseDetector>(detector),
            absoluteBeamOrigin,
            absoluteBeamAttitude,
            currentPulseNumber,
            currentGpsTime,
            writeWaveform,
            calcEchowidth,
            (allMeasurements == nullptr) ?
                nullptr : allMeasurements.get(),
            (allMeasurementsMutex == nullptr) ?
                nullptr : allMeasurementsMutex.get(),
            (cycleMeasurements == nullptr) ?
                nullptr : cycleMeasurements.get(),
            (cycleMeasurementsMutex == nullptr) ?
                nullptr : cycleMeasurementsMutex.get(),
            legIndex
        )
    );
    if(status){
        pool.notify();
        if(status==1){
            dropper = dropper.emptyClone(); // No mutating budding
        }
        else if(status==2){ // Thread pool is full
            // Seq-do task chunk
            dropper.drop(
                apMatrix,
                randGen1,
                randGen2,
                intersectionHandlingNoiseSource
            );
        }
    }
}
