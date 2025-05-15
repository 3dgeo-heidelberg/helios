#include <iostream>

#include "RotateFilter.h"
#include <boost/variant/get.hpp>

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
