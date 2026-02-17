#include <helios/filems/serialization/SerialIO.h>
#include <helios/filems/serialization/SerialSceneWrapper.h>
#include <helios/util/logger/logging.hpp>

// ***  READ / WRITE  *** //
// ********************** //
void
SerialSceneWrapper::writeScene(std::string const& path)
{
  std::stringstream ss;
  ss << "Writing serial scene wrapper object to " << path << " ...";
  logging::INFO(ss.str());
  SerialIO::getInstance()->write<SerialSceneWrapper>(path, this);
}

SerialSceneWrapper*
SerialSceneWrapper::readScene(std::string const& path)
{
  std::stringstream ss;
  ss << "Reading serial scene wrapper object from " << path << " ...";
  logging::INFO(ss.str());
  return SerialIO::getInstance()->read<SerialSceneWrapper>(path);
}
