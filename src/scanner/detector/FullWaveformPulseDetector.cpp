#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "FullWaveformPulseDetector.h"
#include <filems/facade/FMSFacade.h>
#include <filems/write/comps/ZipSyncFileWriter.h>
#include <logging.hpp>

using helios::filems::ZipSyncFileWriter;
using helios::filems::SimpleSyncFileWriter;

using namespace std;


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<AbstractDetector> FullWaveformPulseDetector::clone(){
    std::shared_ptr<AbstractDetector> fwpd =
        std::make_shared<FullWaveformPulseDetector>(
            scanner,
            cfg_device_accuracy_m,
            cfg_device_rangeMin_m
        );
    _clone(fwpd);
    return fwpd;
}
void FullWaveformPulseDetector::_clone(std::shared_ptr<AbstractDetector> ad){
    AbstractDetector::_clone(ad);
    // Below not used atm because class was modified when implementing filems
    //FullWaveformPulseDetector *fwpd = (FullWaveformPulseDetector *) ad.get();
}

// ***  M E T H O D S  *** //
// *********************** //
void FullWaveformPulseDetector::applySettings(shared_ptr<ScannerSettings> & settings) {
	AbstractDetector::applySettings(settings); // calls empty function
}

void FullWaveformPulseDetector::shutdown() {
	AbstractDetector::shutdown();
	if(fms != nullptr && scanner->isWriteWaveform())
        fms->write.finishFullWaveformWriter();
}