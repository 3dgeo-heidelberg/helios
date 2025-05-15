#pragma once

#include <stdexcept>
#include <string>

using std::runtime_error;
using std::string;

namespace rigidmotion {
/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Base class for rigid motion exceptions
 */
class RigidMotionException : public runtime_error
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Rigid motion exception constructor
   * @param msg Rigid motion exception message
   */
  RigidMotionException(string const msg = "")
    : runtime_error(msg)
  {
  }
  virtual ~RigidMotionException() = default;
};
}
