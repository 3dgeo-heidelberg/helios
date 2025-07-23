#pragma once

#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class providing util static methods for dealing with date and time
 *  operations
 */
class DateTimeUtils
{
private:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  DateTimeUtils() = delete;
  virtual ~DateTimeUtils() = default;

public:
  // ***  DATE and TIME UTILS  *** //
  // ***************************** //
  /**
   * @brief Convert from "YYYY-MM-DD hh:mm:ss" string to milliseconds
   * @param str Time "YYYY-MM-DD hh:mm:ss" string itself
   * @return The datetime string converted to milliseconds
   */
  static long dateTimeStrToSeconds(std::string const str);
};
