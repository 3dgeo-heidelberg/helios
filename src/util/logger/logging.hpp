#ifndef __LOGGING_HPP__
#define __LOGGING_HPP__

//see https://gist.github.com/kevinkreiser/39f2e39273c625d96790

#define LOGGING_LEVEL_ALL
/*
Test this with something like:
g++ -std=c++11 -x c++ -pthread -DLOGGING_LEVEL_ALL -DTEST_LOGGING logging.hpp -o logging_test
./logging_test
*/

#include <string>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <mutex>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <logging_common.hpp>
#include <logger.hpp>
#include <std_out_logger.hpp>
#include <file_logger.hpp>
#include <full_logger.hpp>
#include <logger_factory.hpp>
#include <logging_creation.hpp>

/**
 * @file logging.hpp
 *
 * Logging source hub
 */

namespace logging {

//returns formated to: 'year/mo/dy hr:mn:sc.xxxxxx'
/**
 * @brief Obtain current timestamp with format: "yy/mm/dd HH:MM:SS.xxxxxx"
 * @return String timestamp with format: "yy/mm/dd HH:MM:SS.xxxxxx"
 */
inline std::string timestamp() {
    //get the time
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(tp);
    std::tm gmt{};
    #ifdef _OS_WINDOWS_
    gmtime_s(&gmt, &tt);
    #else
    gmtime_r(&tt, &gmt);
    #endif
    std::chrono::duration<double> fractional_seconds =
      (tp - std::chrono::system_clock::from_time_t(tt)) +
      std::chrono::seconds(gmt.tm_sec);
    //format the string
    std::string buffer("year/mo/dy hr:mn:sc.xxxxxx");
    sprintf(
        &buffer.front(),
        "%04d/%02d/%02d %02d:%02d:%09.6f",
        gmt.tm_year + 1900,
        gmt.tm_mon + 1,
        gmt.tm_mday,
        gmt.tm_hour,
        gmt.tm_min,
        fractional_seconds.count()
    );
    return buffer;
  }


//statically get a factory
/**
 * @brief Obtain a logger factory singleton instance
 * @return Logger factory singleton instance
 */
inline logger_factory& get_factory() {
    static logger_factory factory_singleton{};
    return factory_singleton;
}

//get at the singleton
/**
 * @brief Obtain a singleton logger through singleton factory
 * @param config Config for the logger
 * @return Singleton logger
 */
inline logger& get_logger(
    const logging_config_t& config = { {"type", "std_out"}, {"color", ""} }
){
    static std::unique_ptr<logger> singleton(get_factory().produce(config));
    return *singleton;
}

//configure the singleton (once only)
/**
 * @brief Apply given configuration to current logger
 * @param config Configuration to be applied to singleton logger
 */
inline void configure(const logging_config_t& config) {
    get_logger(config);
}

//statically log manually without the macros below
/**
 * @brief Log function wrapper for singleton logger
 * @see logger::log(const std::string&, const log_level)
 */
inline void log(const std::string& message, const log_level level) {
    get_logger().log(message, level);
}

//statically log manually without a level or maybe with a custom one
/**
 * @brief Log function wrapper for singleton logger
 * @see logger::log(const std::string&)
 */
inline void log(const std::string& message) {
    get_logger().log(message);
}

// logging flags
extern bool LOGGING_SHOW_TRACE;
extern bool LOGGING_SHOW_DEBUG;
extern bool LOGGING_SHOW_INFO;
extern bool LOGGING_SHOW_WARN;
extern bool LOGGING_SHOW_ERR;

// logging modes
/**
 * @brief Configure logging mode to make it quiet. Quiet mode means only
 *  errors will be shown
 */
inline void makeQuiet(){
    LOGGING_SHOW_TRACE = false;
    LOGGING_SHOW_DEBUG = false;
    LOGGING_SHOW_INFO = false;
    LOGGING_SHOW_WARN = false;
    LOGGING_SHOW_ERR = true;
}
/**
 * @brief Configure logging mode to make it silent. Silent mode means nothing
 *  will be shown
 */
inline void makeSilent(){
    LOGGING_SHOW_TRACE = false;
    LOGGING_SHOW_DEBUG = false;
    LOGGING_SHOW_INFO = false;
    LOGGING_SHOW_WARN = false;
    LOGGING_SHOW_ERR = false;
}

/**
 * @brief Configure logging mode to make it default. Default mode means only
 *  info and error messages will be shown
 */
inline void makeDefault(){
    LOGGING_SHOW_TRACE = false;
    LOGGING_SHOW_DEBUG = false;
    LOGGING_SHOW_INFO = true;
    LOGGING_SHOW_WARN = false;
    LOGGING_SHOW_ERR = true;
}

/**
 * @brief Configure logging mode to make it verbose. Verbose mode means only
 *  info, warning and error messages will be shown
 *
 */
inline void makeVerbose(){
    LOGGING_SHOW_TRACE = false;
    LOGGING_SHOW_DEBUG = false;
    LOGGING_SHOW_INFO = true;
    LOGGING_SHOW_WARN = true;
    LOGGING_SHOW_ERR = true;
}
/**
 * @brief Configure logging mode to make it verbose level 2. Verbose level 2
 *  mode means all messages will be shown
 */
inline void makeVerbose2(){
    LOGGING_SHOW_TRACE = true;
    LOGGING_SHOW_DEBUG = true;
    LOGGING_SHOW_INFO = true;
    LOGGING_SHOW_WARN = true;
    LOGGING_SHOW_ERR = true;
}


//these standout when reading code
/**
 * @brief Default trace messages logging function
 * @param message Log message
 */
inline void TRACE(const std::string& message) {
    if(!LOGGING_SHOW_TRACE) return;
    get_logger().log(message, log_level::TRACE);
};
/**
 * @brief Default debug messages logging function
 * @param message Log message
 */
inline void DEBUG(const std::string& message) {
    if(!LOGGING_SHOW_DEBUG) return;
    get_logger().log(message, log_level::DEBUG);
};
/**
 * @brief Default info messages logging function
 * @param message Log message
 */
inline void INFO(const std::string& message) {
    if(!LOGGING_SHOW_INFO) return;
    get_logger().log(message, log_level::INFO);
};
/**
 * @brief Default warning messages logging function
 * @param message Log message
 */
inline void WARN(const std::string& message) {
    if(!LOGGING_SHOW_WARN) return;
    get_logger().log(message, log_level::WARN);
};
/**
 * @brief Default error messages logging function
 * @param message Log message
 */
inline void ERR(const std::string& message) {
    if(!LOGGING_SHOW_ERR) return;
    get_logger().log(message, log_level::ERR);
};


}

#endif //__LOGGING_HPP__