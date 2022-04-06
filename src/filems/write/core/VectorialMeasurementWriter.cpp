#include <filems/write/core/VectorialMeasurementWriter.h>
#include <util/HeliosException.h>

#include <sstream>
#include <fstream>

using namespace helios::filems;


// ***  M E T H O D S  *** //
// *********************** //
void VectorialMeasurementWriter::writeMeasurements(
    vector<Measurement> const &measurements
){
    // Check there is a sync file writer
    if(sfw == nullptr){
        throw HeliosException(
            "VectorialMeasurementWriter::writeMeasurements failed because "
            "there was no SyncFileWriter (sfw) available"
        );
    }
    // Check there is a scanner
    else if(scanner == nullptr){
        throw HeliosException(
            "VectorialMeasurementWriter::writeMeasurements failed because "
            "there was no Scanner to associate measurements with"
        );
    }

    // Write measured points to output file
    writeMeasurementsUnsafe(measurements);
}
