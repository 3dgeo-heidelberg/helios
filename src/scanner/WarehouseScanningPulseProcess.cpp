#include <scanner/WarehouseScanningPulseProcess.h>
#include <scanner/Scanner.h>


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
WarehouseScanningPulseProcess::WarehouseScanningPulseProcess(
    std::shared_ptr<Scanner> scanner,
    PulseTaskDropper &dropper,
    PulseWarehouseThreadPool &pool,
    RandomnessGenerator<double> &randGen1,
    RandomnessGenerator<double> &randGen2,
    UniformNoiseSource<double> &intersectionHandlingNoiseSource
#if DATA_ANALYTICS >= 2
    ,std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
) :
    ScanningPulseProcess(scanner),
    dropper(dropper),
    pool(pool),
    randGen1(randGen1),
    randGen2(randGen2),
    intersectionHandlingNoiseSource(intersectionHandlingNoiseSource)
#if DATA_ANALYTICS >= 2
   ,pulseRecorder(pulseRecorder)
#endif
        {
            if(pool.getPoolSize() > 0){ // Parallel computation
        handler = [&] (SimulatedPulse const &sp) -> void {
            handlePulseComputationParallel(sp);
        };
    }
    else{ // Sequential computation
        handler = [&] (SimulatedPulse const &sp) -> void {
            handlePulseComputationSequential(sp);
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
#if DATA_ANALYTICS >= 2
       ,pulseRecorder
#endif
    );

    // Assist thread pool with pending tasks (on WarehouseThreadPool::join atm)
    shared_ptr<PulseTaskDropper> task;
    while( (task=pool.get()) != nullptr){
        (*task)(
            apMatrix, randGen1, randGen2, intersectionHandlingNoiseSource
#if DATA_ANALYTICS >= 2
            ,pulseRecorder
#endif
        );
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
    SimulatedPulse const &sp
){
    // Sequential pulse computation
    shared_ptr<PulseTask> worker = ptf.build(*this, sp);
    (*worker)( // call functor
        apMatrix,
        randGen1,
        randGen2,
        intersectionHandlingNoiseSource
#if DATA_ANALYTICS >= 2
       ,pulseRecorder
#endif
    );

}
void WarehouseScanningPulseProcess::handlePulseComputationParallel(
    SimulatedPulse const &sp
){
    // Submit pulse computation functor to thread pool
    char const status = dropper.tryAdd(
        pool,
        ptf.build(*this, sp)
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
#if DATA_ANALYTICS >= 2
               ,pulseRecorder
#endif
            );
        }
    }
}
