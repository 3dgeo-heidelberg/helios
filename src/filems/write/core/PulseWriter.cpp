#include <filems/write/core/PulseWriter.h>

using namespace helios::filems;

// ***   M E T H O D S   *** //
// ************************* //
void
PulseWriter::writePulse(PulseRecord const& pulseRecord)
{
  sfw->write(pulseRecord);
}
