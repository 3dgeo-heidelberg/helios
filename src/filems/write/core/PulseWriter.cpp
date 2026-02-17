#include <helios/filems/write/core/PulseWriter.h>

// ***   M E T H O D S   *** //
// ************************* //
void
helios::filems::PulseWriter::writePulse(PulseRecord const& pulseRecord)
{
  sfw->write(pulseRecord);
}
