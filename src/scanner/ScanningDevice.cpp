#include <ScanningDevice.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScanningDevice::ScanningDevice(
    std::string const id,
    double const beamDiv_rad,
    glm::dvec3 const beamOrigin,
    Rotation const beamOrientation,
    double const pulseLength_ns,
    double const averagePower_w,
    double const beamQuality,
    double const efficiency,
    double const receiverDiameter_m,
    double const atmosphericVisibility_km,
    double const wavelength_m
) :
    id(id),
    headRelativeEmitterPosition(beamOrigin),
    headRelativeEmitterAttitude(beamOrientation),
    beamDivergence_rad(beamDiv_rad),
    pulseLength_ns(pulseLength_ns),
    averagePower_w(averagePower_w),
    beamQuality(beamQuality),
    efficiency(efficiency),
    receiverDiameter_m(receiverDiameter_m),
    visibility_km(atmosphericVisibility_km),
    wavelength_m(wavelength_m)
{
    configureBeam();
    atmosphericExtinction = calcAtmosphericAttenuation();
    cached_Dr2 = receiverDiameter_m * receiverDiameter_m;
}

ScanningDevice::ScanningDevice(ScanningDevice &scdev){
    // TODO Rethink : Implement
}

// ***  M E T H O D S  *** //
// *********************** //
void ScanningDevice::configureBeam(){
    cached_Bt2 = beamDivergence_rad * beamDivergence_rad;
    beamWaistRadius = (beamQuality * wavelength_m) /
        (M_PI * beamDivergence_rad);
}

// Simulate energy loss from aerial particles (Carlsson et al., 2001)
double ScanningDevice::calcAtmosphericAttenuation() const {
    double q;
    double const lambda = wavelength_m * 1e9;
    double const Vm = visibility_km;

    if (lambda < 500 && lambda > 2000) {
        // Do nothing if wavelength is outside range, approximation will be bad
        return 0;
    }

    if (Vm > 50) q = 1.6;
    else if (Vm > 6 && Vm < 50) q = 1.3;
    else q = 0.585 * pow(Vm, 0.33);

    return (3.91 / Vm) * pow((lambda / 0.55), -q);
}
