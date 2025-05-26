#pragma once

#include <exception>
#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Base class for Helios exceptions
 */
class HeliosException : public std::exception
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Helios exception message
   */
  std::string const msg;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Helios exception constructor
   * @param msg Helios exception message
   * @see HeliosException::msg
   */
  HeliosException(std::string const& msg = "")
    : msg(msg)
  {
  }
  virtual ~HeliosException() = default;

  /**
   * @brief Helios exception overriding of what method
   * @return Helios exception message
   * @see std::exception::what
   * @see HeliosException::msg
   */
  const char* what() const noexcept override { return msg.c_str(); }
};
