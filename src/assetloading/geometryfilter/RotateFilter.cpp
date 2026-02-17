#include <iostream>

#include <boost/variant/get.hpp>
#include <helios/assetloading/geometryfilter/RotateFilter.h>

ScenePart*
RotateFilter::run()
{
  if (primsOut == nullptr) {
    return nullptr;
  }

  if (useLocalRotation) {
    primsOut->mRotation = localRotation.applyTo(primsOut->mRotation);
  } else {
    Rotation rotation = boost::get<Rotation>(params["rotation"]);
    primsOut->mRotation = rotation.applyTo(primsOut->mRotation);
  }

  return primsOut;
}
