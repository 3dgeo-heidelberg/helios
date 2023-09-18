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
    double const incidenceAngle_rad,
    double const targetRange_m,
    double const targetArea_m2,
    double const radius_m,
    double const bdrf,
    double const crossSection,
    double const receivedPower
){
    std::unique_lock<std::mutex> lock(intensityCalcMutex);
    intensityCalc->push(std::vector<double>({
        incidenceAngle_rad, targetRange_m, targetArea_m2, radius_m, bdrf,
        crossSection, receivedPower
    }));
}

#endif
