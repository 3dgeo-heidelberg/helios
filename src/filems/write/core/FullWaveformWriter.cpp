#include <helios/filems/write/core/FullWaveformWriter.h>

// ***   M E T H O D S   *** //
// ************************* //
void
helios::filems::FullWaveformWriter::writeFullWaveform(
  FullWaveform const& fullWaveform)
{
  sfw->write(fullWaveform);
}
