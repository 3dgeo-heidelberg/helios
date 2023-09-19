#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_PulseRecorder.h>

using namespace helios::analytics;


// ***  RECORDER METHODS  *** //
// ************************** //
bool HDA_PulseRecorder::isAnyBufferOpen(){
    bool anyOpen = false;
    anyOpen |= intensityCalc->isOpen();
    return anyOpen;
}

void HDA_PulseRecorder::openBuffers(){
    // Open subray related buffers
    size_t const maxSize = 256;
    std::string const sep = ",";
    intensityCalc = std::make_shared<HDA_RecordBuffer<std::vector<double>>>(
        craftOutputPath("intensity_calc.csv"),
        maxSize,
        sep,
        true  // vectorial flag
    );
}

void HDA_PulseRecorder::closeBuffers(){
    // Close subray buffers
    std::unique_lock<std::mutex> lock(intensityCalcMutex);
    intensityCalc->close();
}


// ***  RECORD METHODS  *** //
// ************************ //
void HDA_PulseRecorder::recordIntensityCalculation(
    std::vector<double> const &record
){
    std::unique_lock<std::mutex> lock(intensityCalcMutex);
    intensityCalc->push(record);
}
void HDA_PulseRecorder::recordIntensityCalculation(
    std::vector<std::vector<double>> const &records
){
    std::unique_lock<std::mutex> lock(intensityCalcMutex);
    for(std::vector<double> const & record : records) {
        intensityCalc->push(record);
    }
}

#endif
