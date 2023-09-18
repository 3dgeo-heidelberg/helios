#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_PulseRecorder.h>

using namespace helios::analytics;


// ***  RECORDER METHODS  *** //
// ************************** //
bool HDA_PulseRecorder::isAnyBufferOpen(){
    bool anyOpen = false;
    anyOpen |= incidenceAngle_rad->isOpen();
    return anyOpen;
}

void HDA_PulseRecorder::openBuffers(){
    // Open subray related buffers
    incidenceAngle_rad = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("incidence_angle_rad.csv")
    );
}

void HDA_PulseRecorder::closeBuffers(){
    // Close subray buffers
    incidenceAngle_rad->close();
}


// ***  RECORD METHODS  *** //
// ************************ //
void HDA_PulseRecorder::recordIncidenceAngle(double const _incidenceAngle_rad){
    std::unique_lock<std::mutex> lock(incidenceAngle_rad_mutex);
    incidenceAngle_rad->push(_incidenceAngle_rad);
}

#endif
