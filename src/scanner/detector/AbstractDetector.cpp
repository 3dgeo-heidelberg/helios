#include <scanner/detector/AbstractDetector.h>
#include <filems/facade/FMSFacade.h>
#include <logging.hpp>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
void AbstractDetector::_clone(std::shared_ptr<AbstractDetector> ad){
    ad->scanner = scanner; // Reference pointer => copy pointer, not object
    ad->cfg_device_accuracy_m = cfg_device_accuracy_m;
    ad->cfg_device_rangeMin_m = cfg_device_rangeMin_m;
    ad->cfg_device_rangeMax_m = cfg_device_rangeMax_m;
    ad->fms = fms;
    ad->pcloudYielder = pcloudYielder;
    ad->fwfYielder = fwfYielder;
}


// ***  M E T H O D S  *** //
// *********************** //
void AbstractDetector::shutdown() {
    if(fms != nullptr) {
        fms->write.finishMeasurementWriter();
        fms->write.finishTrajectoryWriter();
        if (scanner->isWriteWaveform()) fms->write.finishFullWaveformWriter();
    }
}
void AbstractDetector::onLegComplete(){
    if(pcloudYielder != nullptr) pcloudYielder->yield();
    if(fwfYielder != nullptr && scanner->isWriteWaveform())fwfYielder->yield();
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void AbstractDetector::setFMS(std::shared_ptr<FMSFacade> fms) {
    this->fms = fms;
    if(fms != nullptr){
        pcloudYielder = std::make_shared<PointcloudYielder>(fms->write);
        if(scanner->isWriteWaveform()) fwfYielder =
            std::make_shared<FullWaveformYielder>(fms->write);
        if(scanner->isWritePulse()) pulseRecordYielder =
            std::make_shared<PulseRecordYielder>(fms->write);
    }
}
