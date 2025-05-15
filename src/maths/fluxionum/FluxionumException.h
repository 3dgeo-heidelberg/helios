#pragma once

#include <stdexcept>
#include <string>

namespace fluxionum {

using std::runtime_error;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Base class for fluxionum exceptions
 */
class FluxionumException : public runtime_error
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Fluxionum exception constructor
   * @param msg Fluxionum exception message
   */
  FluxionumException(std::string const msg = "")
    : runtime_error(msg)
  {
  }
  virtual ~FluxionumException() = default;
};

}
