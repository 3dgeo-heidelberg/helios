#pragma once

#include <glm/glm.hpp>

namespace boost {
namespace serialization {

/**
 * @brief Serialize a 2 components vector from GLM library to a stream of
 *  bytes
 * @tparam Archive Type of rendering
 * @param ar Specific rendering for the stream of bytes
 * @param vec Vector of 2 components to be serialized
 * @param version Version number for the 2 components vector class
 */
template<class Archive>
void
serialize(Archive& ar, glm::dvec2& vec, const unsigned int version)
{
  ar & vec.x;
  ar & vec.y;
}

/**
 * @brief Serialize a 3 components vector from GLM library to a stream of
 *  bytes
 * @tparam Archive Type of rendering
 * @param ar Specific rendering for the stream of bytes
 * @param vec Vector of 3 components to be serialized
 * @param version Version number for the 3 components vector class
 */
template<class Archive>
void
serialize(Archive& ar, glm::dvec3& vec, const unsigned int version)
{
  ar & vec.x;
  ar & vec.y;
  ar & vec.z;
}
}
}
