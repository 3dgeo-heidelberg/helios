#include <assetloading/geometryfilter/NullGeometryFilter.h>

// ***  MAIN METHODS  *** //
// ********************** //
ScenePart*
NullGeometryFilter::run()
{
  if (primsOut->sorh != nullptr)
    primsOut->sorh->setDiscardOnReplay(true);
  return primsOut;
}
