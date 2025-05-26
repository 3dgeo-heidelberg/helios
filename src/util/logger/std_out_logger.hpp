#pragma once

#include <logger.hpp>

#include <iostream>
#include <string>

// logger that writes to standard out
/**
 * @brief Class representing a logger capable of writing to standard out stream
 */
class std_out_logger : public logger
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Map of logging levels
   * @see logging::uncolored
   * @see logging::colored
   */
  const std::unordered_map<log_level, std::string, enum_hasher> levels;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  std_out_logger() = delete;
  /**
   * @brief Standard out logger constructor
   * @param config Logger configuration which can be used to specify
   *  either colored or uncolored mode
   */
  std_out_logger(const logging_config_t& config)
    : logger(config)
    , levels(config.find("color") != config.end() ? colored : uncolored)
  {
  }

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @see logger::log(const std::string&, const log_level)
   */
  virtual void log(const std::string& message, const log_level level)
  {
    if (level < LOG_LEVEL_CUTOFF)
      return;
    std::string output;
    output.reserve(message.length() + 64);
    // output.append(timestamp());
    // output.append(levels.find(level)->second);
    output.append(message);
    output.push_back('\n');
    log(output);
  }
  /**
   * @see logger::log(const std::string&)
   */
  virtual void log(const std::string& message)
  {
    // cout is thread safe, to avoid multiple threads interleaving on one line
    // though, we make sure to only call the << operator once on std::cout
    // otherwise the << operators from different threads could interleave
    // obviously we dont care if flushes interleave
    std::cout << message;
    std::cout.flush();
  }
};
