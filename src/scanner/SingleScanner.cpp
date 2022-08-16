#include <scanner/SingleScanner.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SingleScanner::SingleScanner(
    double const beamDiv_rad,
    glm::dvec3 const beamOrigin,
    Rotation const beamOrientation,
    std::list<int> const &pulseFreqs,
    double const pulseLength_ns,
    std::string const id,
    double const averagePower,
    double const beamQuality,
    double const efficiency,
    double const receiverDiameter,
    double const atmosphericVisibility,
    int const wavelength,
    bool const writeWaveform = false,
    bool const calcEchowidth = false,
    bool const fullWaveNoise = false,
    bool const platformNoiseDisabled = false
) :
    Scanner(
        id,
        pulseFreqs,
        writeWaveform,
        calcEchowidth,
        fullWaveNoise,
        platformNoiseDisabled
    ), // TODO Rethink : Pass properly arguments to parent constructor
    scanDev(
        id,
        beamDiv_rad,
        beamOrigin,
        beamOrientation,
        pulseLength_ns,
        averagePower,
        beamQuality,
        efficiency,
        receiverDiameter_m,
        atmosphericVisibility,
        wavelength_m
    )
{
    // Report scanner state through logging system
    logging::INFO(toString());
}

SingleScanner::SingleScanner(SingleScanner &scanner) : Scanner(scanner){
    this->scanDev = scanDev;
}

