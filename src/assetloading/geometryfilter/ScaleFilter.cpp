#include <iostream>

#include <boost/variant/get.hpp>
#include <logger_core.hpp>

#include "ScaleFilter.h"

ScenePart*
ScaleFilter::run()
{
  try {
    double scaleFactor = localScaleFactor;
    if (!useLocalScaleFactor) {
      std::map<std::string, ObjectT>::iterator it = params.find("scale");
      scaleFactor = boost::get<double>(it->second);
    }

    if (scaleFactor != 0) {
      primsOut->mScale = scaleFactor;
    }
  } catch (std::exception& e) {
    LOG_WARN(e.what());
  }
  return primsOut;
}
