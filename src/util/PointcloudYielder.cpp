#include <PointcloudYielder.h>

// ***  YIELD METHODS  *** //
// *********************** //
void PointcloudYielder::yield(){
    // Copy and clear in critical region
    mtx.lock();
    vector<Measurement> _measurements(measurements);
    measurements.clear();
    mtx.unlock();
    // Send temporal copy to writer
    write.writeMeasurementsUnsafe(_measurements);
}