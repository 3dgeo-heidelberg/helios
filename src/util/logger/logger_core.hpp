#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <thread>

#include <iostream>
#include <sstream>
#include <chrono>
#include <fstream>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace logging {

enum class log_level : std::uint8_t {
  TRACE = 0,
  DEBUG = 1,
  INFO  = 2,
  TIME  = 3,
  WARN  = 4,
  ERR   = 5,
  OFF   = 255
};

struct log_event {
  log_level level = log_level::INFO;
  std::string message;
  std::int64_t unix_time = 0;
  std::size_t thread_id_hash = 0;
  const char* file = "";
  int line = 0;
  const char* function = "";
  std::string category;
  std::string logger_name = "helios";
};

struct queue_stats {
  std::size_t size = 0;
  std::size_t capacity = 0;
  std::uint64_t overflow_dropped = 0;
  std::uint64_t shutdown_dropped = 0;
  bool stopping = false;
  log_level min_level = log_level::INFO;
};

struct config {
  std::size_t capacity = 4096;
  log_level min_level = log_level::INFO;
  bool clear_queue = true;
  bool reset_counters = true;
};

void configure(const config& cfg);
void configure_silent();
void shutdown() noexcept;

void set_min_level(log_level level) noexcept;

queue_stats stats() noexcept;
std::size_t size() noexcept;
std::size_t capacity() noexcept;

std::size_t drain(std::vector<log_event>& out, std::size_t max_items);
bool wait_pop(log_event& out, std::chrono::milliseconds timeout);

std::uint64_t overflow_dropped_count() noexcept;
std::uint64_t shutdown_dropped_count() noexcept;
void reset_dropped_counters() noexcept;

bool is_stopped() noexcept;

namespace detail {
  bool submit(log_level level,
              std::string message,
              const char* file,
              int line,
              const char* function,
              const char* category = "",
              const char* logger_name = "helios");
}

}

#define LOG(level, msg) \
  do { \
    ::logging::detail::submit((level), (msg), __FILE__, __LINE__, __func__); \
  } while (0)

#define LOG_TRACE(msg) LOG(::logging::log_level::TRACE, msg)
#define LOG_DEBUG(msg) LOG(::logging::log_level::DEBUG, msg)
#define LOG_INFO(msg)  LOG(::logging::log_level::INFO,  msg)
#define LOG_TIME(msg)  LOG(::logging::log_level::TIME,  msg)
#define LOG_WARN(msg)  LOG(::logging::log_level::WARN,  msg)
#define LOG_ERR(msg)   LOG(::logging::log_level::ERR,   msg)