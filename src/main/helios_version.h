#pragma once

#include <string>

/**
 * @brief Current Helios++ version
 */
extern const char *HELIOS_VERSION;
/**
 * @brief The git hash associated to the source code of HELIOS++
 */
extern const char * HELIOS_GIT_HASH;

/**
 * @brief Obtain current Helios++ version
 * @return Current Helios++ version
 */
const char * getHeliosVersion();

/**
 * @brief Obtain the current HELIOS++ version together with extra information
 *  such as the compilation mode, the python bindings, the data analytics mode,
 *  the PCL binding, and the type of linkage for Boost.
 * @return The current HELIOS++ version together with extra information
 */
std::string getHeliosFullVersion();

