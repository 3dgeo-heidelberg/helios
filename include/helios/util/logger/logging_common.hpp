#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

// TODO: use macros (again) so __FILE__ __LINE__ could be automatically added to
// certain error levels? the log levels we support
/**
 * @brief Logging levels enumeration
 */
enum class log_level : std::uint8_t
{
  TRACE = 0,
  DEBUG = 1,
  INFO = 2,
  TIME = 3,
  WARN = 4,
  ERR = 5
};

struct enum_hasher
{
  template<typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

// all, something in between, none or default to info
#if defined(LOGGING_LEVEL_ALL) || defined(LOGGING_LEVEL_TRACE)
constexpr log_level LOG_LEVEL_CUTOFF = log_level::TRACE;
#elif defined(LOGGING_LEVEL_DEBUG)
constexpr log_level LOG_LEVEL_CUTOFF = log_level::DEBUG;
#elif defined(LOGGING_LEVEL_WARN)
constexpr log_level LOG_LEVEL_CUTOFF = log_level::WARN;
#elif defined(LOGGING_LEVEL_ERROR)
constexpr log_level LOG_LEVEL_CUTOFF = log_level::ERR;
#elif defined(LOGGING_LEVEL_NONE)
constexpr log_level LOG_LEVEL_CUTOFF = log_level::ERR + 1;
#else
constexpr log_level LOG_LEVEL_CUTOFF = log_level::INFO;
#endif

/**
 * @brief Uncolored logging tags by level
 */
const std::unordered_map<log_level, std::string, enum_hasher> uncolored{
  { log_level::ERR, " [ERROR] " },   { log_level::WARN, " [WARN] " },
  { log_level::INFO, " [INFO] " },   { log_level::TIME, " [TIME] " },
  { log_level::DEBUG, " [DEBUG] " }, { log_level::TRACE, " [TRACE] " }
};

/**
 * @brief Colored logging tags by level
 */
const std::unordered_map<log_level, std::string, enum_hasher> colored{
  { log_level::ERR, " \x1b[31;1m[ERROR]\x1b[0m " },
  { log_level::WARN, " \x1b[33;1m[WARN]\x1b[0m " },
  { log_level::INFO, " \x1b[32;1m[INFO]\x1b[0m " },
  { log_level::TIME, " \x1b[36;1m[TIME]\x1b[0m " },
  { log_level::DEBUG, " \x1b[34;1m[DEBUG]\x1b[0m " },
  { log_level::TRACE, " \x1b[37;1m[TRACE]\x1b[0m " }
};

// logger base class, not pure virtual so you can use as a null logger
using logging_config_t = std::unordered_map<std::string, std::string>;
