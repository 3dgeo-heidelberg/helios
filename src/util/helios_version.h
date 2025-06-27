#pragma once

#include <string>

/**
 * @brief Obtain current Helios++ version
 * @return Current Helios++ version
 */
std::string
getHeliosVersion();

/**
 * @brief Obtain the current HELIOS++ version together with extra information
 *  such as the compilation mode, the python bindings, the data analytics mode,
 *  the PCL binding, and the type of linkage for Boost.
 * @return The current HELIOS++ version together with extra information
 */
std::string
getHeliosFullVersion();
