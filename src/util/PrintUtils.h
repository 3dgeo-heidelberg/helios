#pragma once

/**
 * @brief Util print functions
 */

#include <ostream>
#include <glm/glm.hpp>


// ***  OPERATOR <<  *** //
// ********************* //
/**
 * @brief Util to print glm::dvec3 instances
 * @code
 * glm::dvec3 v(0,0,1)
 * std::cout << "My vector: " << v << std::endl;
 * @endcode
 */
std::ostream& operator << (std::ostream& out, glm::dvec3 const &v);