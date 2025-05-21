#pragma once

#include <util/HeliosException.h>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris PEna
 * @version 1.0
 *
 * @brief Class representing the end of reading exception for FMS readers
 *
 * @see HeliosException
 */
class EndOfReadingException : public HeliosException
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief End of reading exception constructor
   * @see HeliosException::HeliosException
   */
  EndOfReadingException(std::string const& msg = "End of reading exception")
    : HeliosException(msg)
  {
  }
  ~EndOfReadingException() override = default;
};

}
}
