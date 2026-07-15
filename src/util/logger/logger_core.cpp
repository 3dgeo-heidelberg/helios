#include <logger_core.hpp>

namespace logging {
namespace{
inline std::int64_t
now_unix_time() noexcept
{
  using namespace std::chrono;
  return duration_cast<microseconds>(
           system_clock::now().time_since_epoch())
    .count();
}

inline std::size_t
thread_hash() noexcept
{
  return std::hash<std::thread::id>{}(std::this_thread::get_id());
}

inline std::uint8_t to_raw(log_level level) noexcept
{
  return static_cast<std::uint8_t>(level);
}

inline log_level from_raw(std::uint8_t value) noexcept
{
  return static_cast<log_level>(value);
}

class log_queue{
private:
  mutable std::mutex mutex_;
  std::condition_variable data_cv_;
  std::condition_variable space_cv_;
  std::deque<log_event> queue_;

  std::size_t capacity_;
  std::chrono::milliseconds fallback_wait_{2};

  std::atomic<bool> stopping_{false};
  std::atomic<std::uint8_t> min_level_{static_cast<std::uint8_t>(log_level::INFO)};
  std::atomic<std::uint64_t> overflow_dropped_{0};
  std::atomic<std::uint64_t> shutdown_dropped_{0};
public:
  explicit log_queue(std::size_t capacity = 4096)
    : capacity_(capacity == 0 ? 1 : capacity)
  {}

void configure(const config& cfg)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    capacity_ = cfg.capacity == 0 ? 1 : cfg.capacity;
    min_level_.store(to_raw(cfg.min_level), std::memory_order_release);
    stopping_.store(false, std::memory_order_release);

    if (cfg.clear_queue) {
      queue_.clear();
    }

    if (cfg.reset_counters) {
      overflow_dropped_.store(0, std::memory_order_relaxed);
      shutdown_dropped_.store(0, std::memory_order_relaxed);
    }

    data_cv_.notify_all();
    space_cv_.notify_all();
  }

  void configure_silent()
  {
    config cfg;
    cfg.capacity = 1;
    cfg.min_level = log_level::OFF;
    cfg.clear_queue = true;
    cfg.reset_counters = true;
    configure(cfg);
  }

  void set_min_level(log_level level) noexcept
  {
    min_level_.store(to_raw(level), std::memory_order_release);
  }

  log_level min_level() const noexcept
  {
    return from_raw(min_level_.load(std::memory_order_acquire));
  }

  bool is_stopped() const noexcept
  {
    return stopping_.load(std::memory_order_acquire);
  }

  std::size_t size() const noexcept
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return queue_.size();
  }

  std::size_t capacity() const noexcept
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return capacity_;
  }

  queue_stats stats() const noexcept
  {
    queue_stats s;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      s.size = queue_.size();
      s.capacity = capacity_;
    }
    s.overflow_dropped =
      overflow_dropped_.load(std::memory_order_relaxed);
    s.shutdown_dropped =
      shutdown_dropped_.load(std::memory_order_relaxed);
    s.stopping = stopping_.load(std::memory_order_acquire);
    s.min_level = min_level();
    return s;
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
      // Synchronous fallback policy:
      // wait briefly for space; if still full, drop and count it.
      space_cv_.wait_for(lock, fallback_wait_, [this]() {
        return queue_.size() < capacity_ ||
               stopping_.load(std::memory_order_acquire);
      });
    }

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
    space_cv_.notify_one();
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

    if (n > 0) {
      space_cv_.notify_all();
    }

    return n;
  }

  void shutdown() noexcept
  {
    stopping_.store(true, std::memory_order_release);
    data_cv_.notify_all();
    space_cv_.notify_all();
  }

  std::uint64_t overflow_dropped_count() const noexcept
  {
    return overflow_dropped_.load(std::memory_order_relaxed);
  }

  std::uint64_t shutdown_dropped_count() const noexcept
  {
    return shutdown_dropped_.load(std::memory_order_relaxed);
  }

  void reset_dropped_counters() noexcept
  {
    overflow_dropped_.store(0, std::memory_order_relaxed);
    shutdown_dropped_.store(0, std::memory_order_relaxed);
  }
private:
  bool enabled(log_level level) const noexcept
  {
    return to_raw(level) >= min_level_.load(std::memory_order_acquire);
  }
};

log_queue& queue_instance()
{
  static log_queue instance;
  return instance;
}
}

void
configure(const config& cfg)
{
  queue_instance().configure(cfg);
}

void
configure_silent()
{
  queue_instance().configure_silent();
}

void
shutdown() noexcept
{
  queue_instance().shutdown();
}

void
set_min_level(log_level level) noexcept
{
  queue_instance().set_min_level(level);
}

queue_stats
stats() noexcept
{
  return queue_instance().stats();
}

std::size_t
size() noexcept
{
  return queue_instance().size();
}

std::size_t
capacity() noexcept
{
  return queue_instance().capacity();
}

std::size_t
drain(std::vector<log_event>& out, std::size_t max_items)
{
  return queue_instance().drain(out, max_items);
}

bool
wait_pop(log_event& out, std::chrono::milliseconds timeout)
{
  return queue_instance().wait_pop(out, timeout);
}

std::uint64_t
overflow_dropped_count() noexcept
{
  return queue_instance().overflow_dropped_count();
}

std::uint64_t
shutdown_dropped_count() noexcept
{
  return queue_instance().shutdown_dropped_count();
}

void
reset_dropped_counters() noexcept
{
  queue_instance().reset_dropped_counters();
}

bool
is_stopped() noexcept
{
  return queue_instance().is_stopped();
}

namespace detail {

bool submit(log_level level,
            std::string message,
            const char* file,
            int line,
            const char* function,
            const char* category,
            const char* logger_name)
{
  log_event ev;
  ev.level = level;
  ev.message = std::move(message);
  ev.unix_time = now_unix_time();
  ev.thread_id_hash = thread_hash();
  ev.file = file;
  ev.line = line;
  ev.function = function;
  ev.category = category ? category : "";
  ev.logger_name = logger_name ? logger_name : "helios";

  return queue_instance().push(std::move(ev));
}
}
}