#pragma once

/**
 * @brief Class representing a logger capable of writing to files and, at the
 *  same time, to standard out
 */
class full_logger : public file_logger{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Full logger constructor
     * @param config
     */
    explicit full_logger(const logging_config_t& config):
        file_logger(config)
    {}

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see file_logger::log(const std::string& message)
     */
    void log(const std::string& message) override{
        std::cout << message;
        file_logger::log(message);
    }
};
