#include <ScanningPulseProcess.h>
#include <AbstractDetector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScanningPulseProcess::ScanningPulseProcess(
    std::shared_ptr<AbstractDetector> detector,
    bool const writeWaveform,
    bool const calcEchowidth,
    std::shared_ptr<std::vector<Measurement>> &allMeasurements,
    std::shared_ptr<std::mutex> &allMeasurementsMutex,
    std::shared_ptr<std::vector<Measurement>> &cycleMeasurements,
    std::shared_ptr<std::mutex> &cycleMeasurementsMutex
) :
    ptf(*(detector->scanner->platform->scene)),
    detector(detector),
    writeWaveform(writeWaveform),
    calcEchowidth(calcEchowidth),
    allMeasurements(allMeasurements),
    allMeasurementsMutex(allMeasurementsMutex),
    cycleMeasurements(cycleMeasurements),
    cycleMeasurementsMutex(cycleMeasurementsMutex)
{}
