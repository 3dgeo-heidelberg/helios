#include <helios_version.h>

#include <sstream>

const char * HELIOS_VERSION = "1.1.1";

const char * HELIOS_GIT_HASH = "0344bf65";

const char * getHeliosVersion(){
    return HELIOS_VERSION;
}

std::string getHeliosFullVersion(){
    std::stringstream ss;
    ss << "HELIOS_" << getHeliosVersion();
#ifdef DEBUG_BUILD
    ss << "_DBG";
#else
    ss << "_REL";
#endif
#ifdef PYTHON_BINDING
    ss << "_pybind";
#endif
#ifdef DATA_ANALYTICS
    ss << "_DAmode";
#endif
#ifdef PCL_BINDING
    ss << "_PCL";
#endif
#ifdef DYNAMIC_BOOST
    ss << "_dynBoost";
#else
    ss << "_staBoost";
#endif
    ss << "\nGitHash: " << HELIOS_GIT_HASH;
    return ss.str();
}
