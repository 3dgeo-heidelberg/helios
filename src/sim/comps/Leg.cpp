#include <Leg.h>
#include <ScanningStrip.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Leg::Leg(
    double const length,
    int const serialId,
    std::shared_ptr<ScanningStrip> strip
):
    length(length),
    serialId(serialId),
    strip(strip)
{
    if(strip != nullptr) strip->safeEmplace(serialId, this);
}

Leg::Leg(Leg &leg){
    this->length = leg.length;
    if(leg.mScannerSettings == nullptr) this->mScannerSettings = nullptr;
    else this->mScannerSettings = std::make_shared<ScannerSettings>(
        *leg.mScannerSettings
    );
    if(leg.mPlatformSettings == nullptr) this->mPlatformSettings = nullptr;
    else this->mPlatformSettings = std::make_shared<PlatformSettings>(
        *leg.mPlatformSettings
    );
    if(leg.mTrajectorySettings == nullptr) this->mTrajectorySettings = nullptr;
    else this->mTrajectorySettings = std::make_shared<TrajectorySettings>(
        *leg.mTrajectorySettings
    );
    serialId = leg.serialId;
    strip = leg.strip;
}
