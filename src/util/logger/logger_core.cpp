#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

#include <logger_core.hpp>

namespace logging {
namespace {
inline std::int64_t
now_unix_time() noexcept
{
  using namespace std::chrono;
  return duration_cast<microseconds>(system_clock::now().time_since_epoch())
    .count();
}

inline std::size_t
thread_hash() noexcept
{
  return std::hash<std::thread::id>{}(std::this_thread::get_id());
}

class LogQueue
{
private:
  mutable std::mutex mutex_;
  std::condition_variable data_cv_;
  std::deque<log_event> queue_;

  std::size_t capacity_;

  std::atomic<bool> stopping_{ false };
  std::atomic<std::uint8_t> min_level_{ static_cast<std::uint8_t>(
    log_level::INFO) };
  std::atomic<std::uint64_t> overflow_dropped_{ 0 };
  std::atomic<std::uint64_t> shutdown_dropped_{ 0 };

public:
  explicit LogQueue(std::size_t capacity = 4096)
    : capacity_(capacity == 0 ? 1 : capacity)
  {
  }

  void configure(const config& cfg)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    capacity_ = cfg.capacity == 0 ? 1 : cfg.capacity;
    min_level_.store(static_cast<std::uint8_t>(cfg.min_level),
                     std::memory_order_release);
    stopping_.store(false, std::memory_order_release);

    if (cfg.clear_queue) {
      queue_.clear();
    }

    overflow_dropped_.store(0, std::memory_order_relaxed);
    shutdown_dropped_.store(0, std::memory_order_relaxed);

    data_cv_.notify_all();
  }

  void configure_silent()
  {
    configure({
      .capacity = 1,
      .min_level = log_level::OFF,
      .clear_queue = true,
    });
  }

  void set_min_level(log_level level) noexcept
  {
    min_level_.store(static_cast<std::uint8_t>(level),
                     std::memory_order_release);
  }

  bool is_stopped() const noexcept
  {
    return stopping_.load(std::memory_order_acquire);
  }

  std::size_t capacity() const noexcept
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return capacity_;
  }

  bool push(log_event ev)
  {
    if (!enabled(ev.level)) {
      return true;
    }

    std::unique_lock<std::mutex> lock(mutex_);

    if (stopping_.load(std::memory_order_acquire)) {
      shutdown_dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }

    if (queue_.size() >= capacity_) {
      overflow_dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }

    queue_.push_back(std::move(ev));
    data_cv_.notify_one();
    return true;
  }

  bool wait_pop(log_event& out, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    data_cv_.wait_for(lock, timeout, [this]() {
      return !queue_.empty() || stopping_.load(std::memory_order_acquire);
    });

    if (queue_.empty()) {
      return false;
    }

    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  std::size_t drain(std::vector<log_event>& out, std::size_t max_items)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    const std::size_t n = std::min<std::size_t>(max_items, queue_.size());
    out.reserve(out.size() + n);

    for (std::size_t i = 0; i < n; ++i) {
      out.push_back(std::move(queue_.front()));
      queue_.pop_front();
    }

    return n;
  }

  void shutdown() noexcept
  {
    stopping_.store(true, std::memory_order_release);
    data_cv_.notify_all();
  }

  std::uint64_t consume_overflow_dropped() noexcept
  {
    return overflow_dropped_.exchange(0, std::memory_order_relaxed);
  }

  std::uint64_t consume_shutdown_dropped() noexcept
  {
    return shutdown_dropped_.exchange(0, std::memory_order_relaxed);
  }

  void reset_dropped_counters() noexcept
  {
    overflow_dropped_.store(0, std::memory_order_relaxed);
    shutdown_dropped_.store(0, std::memory_order_relaxed);
  }

private:
  bool enabled(log_level level) const noexcept
  {
    return static_cast<std::uint8_t>(level) >=
           min_level_.load(std::memory_order_acquire);
  }
};

LogQueue&
log_queue_instance()
{
  static LogQueue instance;
  return instance;
}
}

void
configure(const config& cfg)
{
  log_queue_instance().configure(cfg);
}

void
configure_silent()
{
  log_queue_instance().configure_silent();
}

void
shutdown() noexcept
{
  log_queue_instance().shutdown();
}

void
set_min_level(log_level level) noexcept
{
  log_queue_instance().set_min_level(level);
}

std::size_t
capacity() noexcept
{
  return log_queue_instance().capacity();
}

std::size_t
drain(std::vector<log_event>& out, std::size_t max_items)
{
  return log_queue_instance().drain(out, max_items);
}

bool
wait_pop(log_event& out, std::chrono::milliseconds timeout)
{
  return log_queue_instance().wait_pop(out, timeout);
}

bool
is_stopped() noexcept
{
  return log_queue_instance().is_stopped();
}

dropped_counts
consume_dropped_counts() noexcept
{
  auto& q = log_queue_instance();
  return {
    q.consume_overflow_dropped(),
    q.consume_shutdown_dropped(),
  };
}

namespace detail {

bool
submit(log_level level,
       std::string message,
       const char* file,
       int line,
       const char* function)
{
  log_event ev;
  ev.level = level;
  ev.message = std::move(message);
  ev.unix_time = now_unix_time();
  ev.thread_id_hash = thread_hash();
  ev.file = file;
  ev.line = line;
  ev.function = function;

  return log_queue_instance().push(std::move(ev));
}
}
}
