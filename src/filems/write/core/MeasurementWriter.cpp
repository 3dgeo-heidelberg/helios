#include <filems/write/core/MeasurementWriter.h>
#include <util/HeliosException.h>

using namespace helios::filems;

// ***   M E T H O D S   *** //
// ************************* //
void
MeasurementWriter::writeMeasurement(Measurement const& m)
{
  // Check there is a sync file writer
  if (sfw == nullptr) {
    throw HeliosException(
      "MeasurementWriter::writeMeasurement failed because there was no "
      "SyncFileWriter (sfw) available");

  }
  // Check there is a scanner
  else if (scanner == nullptr) {
    throw HeliosException(
      "MeasurementWriter::writeMeasurement failed because there was no "
      "Scanner to associate measurements with");
  }

  // Write measured point to output file
  writeMeasurementUnsafe(m);
}
