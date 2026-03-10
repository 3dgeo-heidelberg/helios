#include <DateTimeUtils.h>
// #include <iostream>
// #include "logging.hpp"

#include <chrono>
#include <ctime>
#include <stdexcept>

using namespace std::chrono;

namespace {
long
convertUtcTmToSeconds(std::tm* t)
{
#ifdef _WIN32
  std::time_t const tt = _mkgmtime(t);
#else
  std::time_t const tt = timegm(t);
#endif
  return duration_cast<seconds>(
           system_clock::from_time_t(tt).time_since_epoch())
    .count();
}

int
parseTwoDigits(std::string const& str, std::size_t pos)
{
  return std::stoi(str.substr(pos, 2));
}
} // namespace

// ***  DATE and TIME UTILS  *** //
// ***************************** //
long
DateTimeUtils::dateTimeStrToSeconds(std::string const str)
{
  if (str.size() < 19) {
    throw std::invalid_argument("datetime string too short");
  }

  std::tm t = {};
  t.tm_year = stoi(str.substr(0, 4)) - 1900;
  t.tm_mon = stoi(str.substr(5, 2)) - 1;
  t.tm_mday = stoi(str.substr(8, 2));
  t.tm_hour = stoi(str.substr(11, 2));
  t.tm_min = stoi(str.substr(14, 2));
  t.tm_sec = stoi(str.substr(17, 2));
  t.tm_isdst = 0;

  long const utcSeconds = convertUtcTmToSeconds(&t);
  if (str.size() == 19) {
    return utcSeconds;
  }

  std::string const tz = str.substr(19);
  if (tz == "Z") {
    return utcSeconds;
  }

  if (tz.size() == 6 && (tz[0] == '+' || tz[0] == '-') && tz[3] == ':') {
    int const hours = parseTwoDigits(tz, 1);
    int const minutes = parseTwoDigits(tz, 4);
    if (hours > 23 || minutes > 59) {
      throw std::invalid_argument("invalid timezone offset");
    }
    long const offsetSeconds = static_cast<long>((hours * 60 + minutes) * 60);
    return tz[0] == '+' ? utcSeconds - offsetSeconds
                        : utcSeconds + offsetSeconds;
  }

  throw std::invalid_argument("unsupported timezone suffix");
}
