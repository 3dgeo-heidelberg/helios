#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace logging {

enum class log_level : std::uint8_t
{
  TRACE = 0,
  DEBUG = 1,
  INFO = 2,
  TIME = 3,
  WARN = 4,
  ERR = 5,
  OFF = 255
};

struct log_event
{
  log_level level = log_level::INFO;
  std::string message;
  std::int64_t unix_time = 0;
  std::size_t thread_id_hash = 0;
  const char* file = "";
  int line = 0;
  const char* function = "";
};

struct dropped_counts
{
  std::uint64_t overflow = 0;
  std::uint64_t shutdown = 0;
};

struct config
{
  std::size_t capacity = 4096;
  log_level min_level = log_level::INFO;
  bool clear_queue = true;
};

dropped_counts
consume_dropped_counts() noexcept;

void
configure(const config& cfg);
void
configure_silent();
void
shutdown() noexcept;
void
set_min_level(log_level level) noexcept;

std::size_t
drain(std::vector<log_event>& out, std::size_t max_items);
bool
wait_pop(log_event& out, std::chrono::milliseconds timeout);
bool
is_stopped() noexcept;

namespace detail {
// Minimum enabled level, read at every log call site before the message is
// built.
extern std::atomic<std::uint8_t> min_level;

inline bool
is_enabled(log_level level) noexcept
{
  return static_cast<std::uint8_t>(level) >=
         min_level.load(std::memory_order_relaxed);
}

bool
submit(log_level level,
       std::string message,
       const char* file,
       int line,
       const char* function);
}

}

#define LOG(level, msg)                                                        \
  do {                                                                         \
    const ::logging::log_level helios_log_level_ = (level);                    \
    if (::logging::detail::is_enabled(helios_log_level_)) {                    \
      ::logging::detail::submit(                                               \
        helios_log_level_, (msg), __FILE__, __LINE__, __func__);               \
    }                                                                          \
  } while (0)

#define LOG_IS_ENABLED(level_name)                                             \
  (::logging::detail::is_enabled(::logging::log_level::level_name))

#define LOG_TRACE(msg) LOG(::logging::log_level::TRACE, msg)
#define LOG_DEBUG(msg) LOG(::logging::log_level::DEBUG, msg)
#define LOG_INFO(msg) LOG(::logging::log_level::INFO, msg)
#define LOG_TIME(msg) LOG(::logging::log_level::TIME, msg)
#define LOG_WARN(msg) LOG(::logging::log_level::WARN, msg)
#define LOG_ERR(msg) LOG(::logging::log_level::ERR, msg)
