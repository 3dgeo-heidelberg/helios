#pragma once

#include <logging_common.hpp>

#include <mutex>

/**
 * @brief Class providing the base for any logger
 */
class logger
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Mutex to handle concurrent log writes
   */
  std::mutex lock;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  logger() = delete;
  /**
   * @brief Logger constructor
   * @param config Logger configuration
   */
  logger(const logging_config_t& config) {};
  virtual ~logger() {};

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Handle a log entry considerings its level
   * @param message Content for log entry
   * @param level Level associated with the entry
   */
  virtual void log(const std::string& message, const log_level level) {};
  /**
   * @brief Handle a log entry
   * @param message Content for log entry
   */
  virtual void log(const std::string& message) {};
};
