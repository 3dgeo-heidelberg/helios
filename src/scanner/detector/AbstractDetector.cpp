#include <scanner/detector/AbstractDetector.h>
#include <logging.hpp>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
void AbstractDetector::_clone(std::shared_ptr<AbstractDetector> ad){
    ad->scanner = scanner; // Reference pointer => copy pointer, not object
	if(mBuffer == nullptr) ad->mBuffer = nullptr;
    else ad->mBuffer = std::make_shared<MeasurementsBuffer>(*mBuffer);
    ad->cfg_device_accuracy_m = cfg_device_accuracy_m;
    ad->cfg_device_rangeMin_m = cfg_device_rangeMin_m;
    ad->fms = fms;
}

// ***  M E T H O D S  *** //
// *********************** //

void AbstractDetector::shutdown() {
    fms->write.finishMeasurementWriter();
}