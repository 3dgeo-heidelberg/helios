#include <scanner/SingleScanner.h>
#include <scanner/detector/AbstractDetector.h>

#ifdef PYTHON_BINDING
#include "PyDetectorWrapper.h"
#endif

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
    bool const writeWaveform,
    bool const calcEchowidth,
    bool const fullWaveNoise,
    bool const platformNoiseDisabled
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
        receiverDiameter,
        atmosphericVisibility,
        wavelength  / 1000000000.0
    )
{
    // Report scanner state through logging system
    logging::INFO(toString());
}

SingleScanner::SingleScanner(SingleScanner &scanner) :
    Scanner(scanner),
    scanDev(scanner.scanDev)
{}

// ***   C L O N E   *** //
// ********************* //
std::shared_ptr<Scanner> SingleScanner::clone() {
    std::shared_ptr<Scanner> scanner = std::make_shared<SingleScanner>(*this);
    _clone(*scanner);
    return scanner;
}

void SingleScanner::_clone(Scanner &sc) const{
    // Call parent clone method
    Scanner::_clone(sc);
    // Clone attributes from SingleScanner class itself
    //SingleScanner & ssc = static_cast<SingleScanner &>(sc);  // Not used
    //sc.scanDev = scanDev;  // Not necessary because of copy constructor call
}


// ***   M E T H O D S   *** //
// ************************* //
void SingleScanner::applySettings(std::shared_ptr<ScannerSettings> settings){
    // Configure scanner and scanning device
    setPulseFreq_Hz(settings->pulseFreq_Hz);
    setActive(settings->active);
    setBeamDivergence(settings->beamDivAngle, 0);
    trajectoryTimeInterval_ns = settings->trajectoryTimeInterval*1000000000.0;
    scanDev.configureBeam();

    // Configure other components
    detector->applySettings(settings);
    scannerHead->applySettings(settings);
    beamDeflector->applySettings(settings);
}

void SingleScanner::prepareDiscretization(){
    numTimeBins = getPulseLength_ns(0) / FWF_settings.binSize_ns;
    time_wave = vector<double>(numTimeBins);
    peakIntensityIndex = calcTimePropagation(time_wave, numTimeBins);
}

int SingleScanner::calcTimePropagation(
    std::vector<double> & timeWave, int const numBins
) const {
    double const step = FWF_settings.binSize_ns;
    //double const tau = (getPulseLength_ns() * 0.5) / 3.5;  // Too many ops.
    double const tau = getPulseLength_ns(0) / 7.0;  // Just one op.
    double t = 0;
    double t_tau = 0;
    double pt = 0;
    double peakValue = 0;
    int peakIndex = 0;

    for (int i = 0; i < numBins; ++i) {
        t = i * step;
        t_tau = t / tau;
        pt = (t_tau * t_tau) * exp(-t_tau);
        timeWave[i] = pt;
        if (pt > peakValue) {
            peakValue = pt;
            peakIndex = i;
        }
    }

    return peakIndex;
}

double SingleScanner::calcFootprintArea(double const distance) const {
    return (M_PI * distance * distance * getBt2(0)) / 4;
}

Rotation SingleScanner::calcAbsoluteBeamAttitude() const{
    Rotation mountRelativeEmitterAttitude =
        scannerHead->getMountRelativeAttitude()
            .applyTo(getHeadRelativeEmitterAttitude(0));
    return platform->getAbsoluteMountAttitude()
        .applyTo(mountRelativeEmitterAttitude)
        .applyTo(beamDeflector->getEmitterRelativeAttitude());
}
