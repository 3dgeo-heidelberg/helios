#pragma once

#include <stdexcept>
#include <string>

namespace rigidmotion {
/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Base class for rigid motion exceptions
 */
class RigidMotionException : public std::runtime_error
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Rigid motion exception constructor
   * @param msg Rigid motion exception message
   */
  RigidMotionException(std::string const msg = "")
    : std::runtime_error(msg)
  {
  }
  virtual ~RigidMotionException() = default;
};
}
