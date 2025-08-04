#pragma once

// Include boost ASIO preventing windows conflicts ---
#if defined(_WIN32) || defined(_WIN64) // If using windows
// Check WIN32 LEAN AND MEAN is specified, error otherwise
#ifndef WIN32_LEAN_AND_MEAN
#error WIN32 LEAN AND MEAN MACRO WAS NOT DEFINED
#endif
#include <SDKDDKVer.h>
#include <boost/asio.hpp>

#else // If not using windows
#include <boost/asio.hpp>

#endif
// --- Include boost ASIO preventing windows conflicts
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <logging.hpp>
#include <sstream>

/**
 * @version 1.0
 * @brief Base class providing core implementation of a thread pool to deal
 *  with multi threading tasks
 */
class ThreadPool
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Instance of boost input/output service for asynchronous data
   *  processing
   */
  boost::asio::io_context io_context_;
  /**
   * @brief Instance of work guard to report the io service when it has
   *  pending tasks
   * @see ThreadPool::io_context_
   */
  using work_guard_type =
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;
  work_guard_type work_guard_;
  /**
   * @brief Size of thread pool (number of threads)
   */
  std::size_t pool_size;
  /**
   * @brief Group of threads
   */
  boost::thread_group threads_;
  /**
   * @brief True when the thread pool has been finished, false otherwise.
   */
  bool finished;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Thread pool constructor
   * @see ThreadPool::pool_size
   */
  explicit ThreadPool(std::size_t const _pool_size)
    : work_guard_(boost::asio::make_work_guard(io_context_))
    , pool_size(_pool_size)
    , finished(false)
  {
    // Create threads
    for (std::size_t i = 0; i < pool_size; ++i) {
      threads_.create_thread([&]() -> boost::asio::io_context::count_type {
        return io_context_.run();
      });
    }
  }

  virtual ~ThreadPool()
  {
    // Force all threads to return from io_context_::run().
    io_context_.stop();

    // Suppress all exceptions.
    if (!finished) {
      try {
        threads_.join_all(); // Legacy mode
      } catch (const std::exception&) {
      }
    }
  }

public:
  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Obtain the thread pool size
   * @return Thread pool size
   * @see ThreadPool::pool_size
   */
  virtual inline std::size_t getPoolSize() const { return pool_size; }
};
