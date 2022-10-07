#include <scanner/MultiScanner.h>
#include <maths/WaveMaths.h>
#include <scanner/detector/AbstractDetector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
// TODO Rethink : Restore and complete implementation below
/*MultiScanner::MultiScanner(MultiScanner &scanner) :
    Scanner(scanner),
    scanDevs(std::move(scanner.scanDevs))
{}


// ***   C L O N E   *** //
// ********************* //
std::shared_ptr<Scanner> MultiScanner::clone(){
    std::shared_ptr<Scanner> scanner = std::make_shared<MultiScanner>(*this);
    _clone(*scanner);
    return scanner;
}

void MultiScanner::_clone(Scanner &sc) const{
    // Call parent clone method
    MultiScanner::_clone(sc);
    // Clone attributes from MultiScanner class itself
    //MultiScanner &ssc = static_cast<MultiScanner &>(sc);  // Not used
    //ssc.scanDevs = scanDevs;  // Already handled by copy constructor
}

// ***   M E T H O D S   *** //
// ************************* //
void MultiScanner::applySettings(
    std::shared_ptr<ScannerSettings> settings, size_t const idx
){
    // Configure scanner
    setPulseFreq_Hz(settings->pulseFreq_Hz);
    setActive(settings->active);
    setBeamDivergence(settings->beamDivAngle, 0);
    trajectoryTimeInterval_ns = settings->trajectoryTimeInterval*1e9;

    // Configure scanning devices and their components
    size_t const numScanDevs = scanDevs.size();
    for(size_t i = 0 ; i < numScanDevs ; ++i){
        scanDevs[i].configureBeam();
        getDetector(i)->applySettings(settings);
        getScannerHead(i)->applySettings(settings);
        getBeamDeflector(i)->applySettings(settings);
    }
}

void MultiScanner::prepareDiscretization(size_t const idx){
    setNumTimeBins(
        getPulseLength_ns(idx) / getFWFSettings(idx).binSize_ns,
        idx
    );
    setTimeWave(vector<double>(getNumTimeBins(idx)), idx);
    setPeakIntensityIndex(WaveMaths::calcPropagationTimeLegacy(
        getTimeWave(idx),
        getNumTimeBins(idx),
        getFWFSettings(idx).binSize_ns,
        getPulseLength_ns(idx),
        7.0  // 3.5 too many ops., 7.0 just one op.
    ), idx);
}

double MultiScanner::calcFootprintArea(
    double const distance, size_t const idx
) const{
    return PI_QUARTER * distance * distance * getBt2(idx);
}

Rotation MultiScanner::calcAbsoluteBeamAttitude(size_t const idx){
    return scanDevs[idx].calcAbsoluteBeamAttitude(
        platform->getAbsoluteMountAttitude()
    );
}*/
