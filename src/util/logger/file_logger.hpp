#pragma once

//TODO: add log rolling
//logger that writes to file
/**
 * @brief Class representing a logger capable of writing to files
 */
class file_logger : public logger {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Name of output file
     */
    std::string file_name;
    /**
     * @brief Output file stream
     */
    std::ofstream file;
    /**
     * @brief Reopen interval in seconds
     */
    std::chrono::seconds reopen_interval;
    /**
     * @brief Time point when last reopen took place
     */
    std::chrono::system_clock::time_point last_reopen;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    file_logger() = delete;
    /**
     * @brief File logger constructor
     * @param config File logger configuration, which can be used to specify
     *  reopen file name and reopen interval
     */
    file_logger(const logging_config_t& config):logger(config) {
        //grab the file name
        auto name = config.find("file_name");
        if(name == config.end())
            throw std::runtime_error("No output file provided to file logger");
        file_name = name->second;

        //if we specify an interval
        reopen_interval = std::chrono::seconds(300);
        auto interval = config.find("reopen_interval");
        if(interval != config.end()){
            try {
                reopen_interval = std::chrono::seconds(
                    std::stoul(interval->second)
                );
            }
            catch(...) {
                throw std::runtime_error(
                    interval->second + " is not a valid reopen interval"
                );
            }
        }

        //crack the file open
        reopen();
    }

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see logger::log(const std::string&, const log_level)
     */
    virtual void log(const std::string& message, const log_level level) {
        if(level < LOG_LEVEL_CUTOFF) return;
        std::string output;
        output.reserve(message.length() + 64);
        //output.append(timestamp());
        //output.append(uncolored.find(level)->second);
        output.append(message);
        output.push_back('\n');
        log(output);
    }

    /**
     * @see logger::log(const std::string&)
     */
    void log(const std::string& message) override{
        lock.lock();
        file << message;
        file.flush();
        lock.unlock();
        reopen();
    }
protected:
    /**
     * @brief Reopen the log file in a thread-safe fashion
     */
    void reopen() {
        //TODO: use CLOCK_MONOTONIC_COARSE
        //check if it should be closed and reopened
        auto now = std::chrono::system_clock::now();
        lock.lock();
        if(now - last_reopen > reopen_interval) {
            last_reopen = now;
            try{ file.close(); }catch(...){}
            try {
                file.open(file_name, std::ofstream::out | std::ofstream::app);
                last_reopen = std::chrono::system_clock::now();
            }
            catch(std::exception& e) {
                try{ file.close(); }catch(...){}
                throw e;
            }
        }
        lock.unlock();
    }
};
