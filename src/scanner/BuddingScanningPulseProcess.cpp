#include <scanner/BuddingScanningPulseProcess.h>
#include <FullWaveformPulseDetector.h>
#include <FullWaveformPulseRunnable.h>

#include <memory>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
BuddingScanningPulseProcess::BuddingScanningPulseProcess(
    std::shared_ptr<AbstractDetector> &detector,
    int &currentPulseNumber,
    bool &writeWaveform,
    bool &calcEchowidth,
    std::shared_ptr<std::vector<Measurement>> &allMeasurements,
    std::shared_ptr<std::mutex> &allMeasurementsMutex,
    std::shared_ptr<std::vector<Measurement>> &cycleMeasurements,
    std::shared_ptr<std::mutex> &cycleMeasurementsMutex,
    PulseTaskDropper &dropper,
    PulseThreadPool &pool,
    RandomnessGenerator<double> &randGen1,
    RandomnessGenerator<double> &randGen2,
    UniformNoiseSource<double> &intersectionHandlingNoiseSource,
    bool const dynamic
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
    if(pool.getPoolSize() > 0){
        if(pool.isDynamic()){ // Dynamic chunk schedule
            handler = [&] (
                unsigned int const legIndex,
                glm::dvec3 &absoluteBeamOrigin,
                Rotation &absoluteBeamAttitude,
                double const currentGpsTime
                ) -> void {
                handlePulseComputationParallelDynamic(
                    legIndex,
                    absoluteBeamOrigin,
                    absoluteBeamAttitude,
                    currentGpsTime
                    );
            };
        }
        else{ // Static chunk schedule
            handler = [&] (
                unsigned int const legIndex,
                glm::dvec3 &absoluteBeamOrigin,
                Rotation &absoluteBeamAttitude,
                double const currentGpsTime
            ) -> void {
                handlePulseComputationParallelStatic(
                    legIndex,
                    absoluteBeamOrigin,
                    absoluteBeamAttitude,
                    currentGpsTime
                );
            };
        }
    }
    else{
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
#ifdef BUDDING_METRICS
    ofsBudding.open("budding_metrics.csv", std::ios_base::out);
    ofsBudding.setf(std::ios::fixed);
    ofsBudding.precision(6);
#endif
}

// ***  PULSE COMPUTATION  *** //
// *************************** //
void BuddingScanningPulseProcess::onLegComplete(){
    // If there is a pending chunk of tasks, sequentially compute it
    std::vector<std::vector<double>> apMatrix;
    dropper.drop(
        apMatrix,
        randGen1,
        randGen2,
        intersectionHandlingNoiseSource
    );
#ifdef BUDDING_METRICS
    ofsBudding.flush();
#endif
    pool.idleTimer.releaseStart();
    idleTimer.releaseStart();
}
void BuddingScanningPulseProcess::onSimulationFinished(){
#ifdef BUDDING_METRICS
    ofsBudding.close();
#endif
}

// ***  INNER PULSE COMPUTATION  *** //
// ********************************* //
void BuddingScanningPulseProcess::handlePulseComputationSequential(
    unsigned int const legIndex,
    glm::dvec3 &absoluteBeamOrigin,
    Rotation &absoluteBeamAttitude,
    double const currentGpsTime
){
    // Sequential pulse computation
    std::vector<std::vector<double>> apMatrix;
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
void BuddingScanningPulseProcess::handlePulseComputationParallelDynamic(
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
    if(status==1){ // Dropper successfully posted to thread pool
        if(idleTimer.hasStarted()){
            long const idleNanos = idleTimer.getElapsedNanos();
            idleTimer.releaseStart();
#ifdef BUDDING_METRICS
            bool debugBuddingMetrics;
#endif
            dropper = dropper.evolve(
#ifdef BUDDING_METRICS
                debugBuddingMetrics,
#endif
                lastIdleNanos, idleNanos, idleTh, idleEps
            );
            lastIdleNanos = idleNanos;
            // Metric for debugging below (-DBUDDING_METRICS)
#ifdef BUDDING_METRICS
            if(debugBuddingMetrics){
                ofsBudding  << idleNanos << ","
                            << dropper.getMaxTasks() << ","
                            << dropper.getDelta1() << ","
                            << dropper.getDelta2() << ","
                            << (int) dropper.getLastSign() << "\n"
                ;
            }
#endif
        }
        else{
            dropper = dropper.emptyClone(); // No mutating budding
        }
    }
    else if(status==2){ // Thread pool is full
        // Compute idle time (time between first idle and full work)
        if(pool.idleTimer.hasStarted()){
            pool.idleTimer.stop();
            idleTimer.synchronize(pool.idleTimer);
            pool.idleTimer.releaseStart();
        }
        // Seq-do popped task
        std::vector<std::vector<double>> apMatrix;
        (*dropper.popTask())(
            apMatrix,
            randGen1,
            randGen2,
            intersectionHandlingNoiseSource
        );
    }
}

void BuddingScanningPulseProcess::handlePulseComputationParallelStatic(
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
    if(status==1){ // Dropper successfully posted to thread pool
#ifdef BUDDING_METRICS
        if(idleTimer.hasStarted()){
            long const idleNanos = idleTimer.getElapsedNanos();
            idleTimer.releaseStart();
            if(std::abs(idleNanos-lastIdleNanos) > idleEps){
                lastIdleNanos = idleNanos;
                ofsBudding  << idleNanos << ","
                            << dropper.getMaxTasks() << ","
                            << dropper.getDelta1() << ","
                            << dropper.getDelta2() << ","
                            << (int) dropper.getLastSign() << "\n"
                ;
            }
        }
#endif
        dropper = dropper.emptyClone(); // No mutating budding
    }
    else if(status==2){ // Thread pool is full
#ifdef BUDDING_METRICS
        // Compute idle time (time between first idle and full work)
        if(pool.idleTimer.hasStarted()){
            pool.idleTimer.stop();
            idleTimer.synchronize(pool.idleTimer);
            pool.idleTimer.releaseStart();
        }
#endif
        // Seq-do popped task
        std::vector<std::vector<double>> apMatrix;
        (*dropper.popTask())(
            apMatrix,
            randGen1,
            randGen2,
            intersectionHandlingNoiseSource
        );
    }
}
