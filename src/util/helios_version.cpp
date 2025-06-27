#include <helios_version.h>

#include <sstream>

std::string
getHeliosVersion()
{
  return HELIOS_VERSION;
}

std::string
getHeliosFullVersion()
{
  std::stringstream ss;
  ss << "Helios v" << getHeliosVersion() << " [" << HELIOS_BUILD_TYPE;
#ifdef DATA_ANALYTICS
  ss << " DAmode";
#endif
#ifdef PCL_BINDING
  ss << " PCL";
#endif
  ss << "]";
  return ss.str();
}
