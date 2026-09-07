#include <filems/facade/FMSFacade.h>
#include <scanner/detector/AbstractDetector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
void
AbstractDetector::_clone(std::shared_ptr<AbstractDetector> ad)
{
  ad->scanner = scanner; // Weak back-reference => copy pointer, not object
  ad->cfg_device_accuracy_m = cfg_device_accuracy_m;
  ad->cfg_device_rangeMin_m = cfg_device_rangeMin_m;
  ad->cfg_device_rangeMax_m = cfg_device_rangeMax_m;
  ad->fms = fms;
  ad->pcloudYielder = pcloudYielder;
  ad->fwfYielder = fwfYielder;
}

// ***  M E T H O D S  *** //
// *********************** //
void
AbstractDetector::shutdown()
{
  if (fms != nullptr) {
    fms->write.finishMeasurementWriter();
    fms->write.finishTrajectoryWriter();
    auto owner = scanner.lock();
    if (owner != nullptr && owner->isWriteWaveform())
      fms->write.finishFullWaveformWriter();
  }
}
void
AbstractDetector::onLegComplete()
{
  if (pcloudYielder != nullptr)
    pcloudYielder->yield();
  auto owner = scanner.lock();
  if (fwfYielder != nullptr && owner != nullptr && owner->isWriteWaveform())
    fwfYielder->yield();
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void
AbstractDetector::setFMS(std::shared_ptr<helios::filems::FMSFacade> fms)
{
  this->fms = fms;
  if (fms != nullptr) {
    pcloudYielder = std::make_shared<PointcloudYielder>(fms->write);
    auto owner = scanner.lock();
    if (owner != nullptr && owner->isWriteWaveform())
      fwfYielder = std::make_shared<FullWaveformYielder>(fms->write);
    if (owner != nullptr && owner->isWritePulse())
      pulseRecordYielder = std::make_shared<PulseRecordYielder>(fms->write);
  }
}
