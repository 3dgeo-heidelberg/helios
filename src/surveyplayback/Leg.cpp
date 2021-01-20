#include "Leg.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
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
}
