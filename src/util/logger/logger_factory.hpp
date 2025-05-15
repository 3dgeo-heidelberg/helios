#pragma once

#include <logging_common.hpp>
#include <logging_creation.hpp>

/**
 * @brief Logger factory class can be used to build loggers
 */
class logger_factory
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Logger creation map which works with function pointers
   */
  std::unordered_map<std::string, logger_creator> creators;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Logger factory constructor
   */
  logger_factory()
  {
    creators.emplace("", [](const logging_config_t& config) -> logger* {
      return new logger(config);
    });
    creators.emplace("std_out", [](const logging_config_t& config) -> logger* {
      return new std_out_logger(config);
    });
    creators.emplace("file", [](const logging_config_t& config) -> logger* {
      return new file_logger(config);
    });
    creators.emplace("full", [](const logging_config_t& config) -> logger* {
      return new full_logger(config);
    });
  }

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Produce a logger
   * @param config Config to be used to build the logger
   * @return Produced logger
   */
  logger* produce(const logging_config_t& config) const
  {
    // grab the type
    auto type = config.find("type");
    if (type == config.end()) {
      throw std::runtime_error(
        "Logging factory configuration requires a type of logger");
    }
    // grab the logger
    auto found = creators.find(type->second);
    if (found != creators.end())
      return found->second(config);
    // couldn't get a logger
    throw std::runtime_error("Couldn't produce logger for type: " +
                             type->second);
  }
};
