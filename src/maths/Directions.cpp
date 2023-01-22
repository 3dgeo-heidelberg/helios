#include <glm/glm.hpp>
#include "Directions.h"

// ***  CONSTANTS  *** //
// ******************* //
// Canonical basis
const glm::dvec3 Directions::right = glm::dvec3(1, 0, 0);
const glm::dvec3 Directions::forward = glm::dvec3(0, 1, 0);
const glm::dvec3 Directions::up = glm::dvec3(0, 0, 1);

// ARINC 705 norm
const glm::dvec3 Directions::yaw = glm::dvec3(0, 0, -1);
const glm::dvec3 Directions::roll = glm::dvec3(0, 1, 0);
const glm::dvec3 Directions::pitch = glm::dvec3(1, 0, 0);
