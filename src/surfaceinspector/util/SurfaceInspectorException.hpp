#pragma once

#include <stdexcept>
#include <string>
#include <surfaceinspector/util/Object.hpp>

using std::runtime_error;
using std::string;

namespace SurfaceInspector {
namespace util {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Base class for surface inspector exceptions
 */
class SurfaceInspectorException
  : public runtime_error
  , Object
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Surface inspector exception constructor
   * @param msg Surface inspector exception message
   */
  SurfaceInspectorException(std::string const msg = "")
    : runtime_error(msg)
  {
  }
  virtual ~SurfaceInspectorException() = default;
};

}
}
