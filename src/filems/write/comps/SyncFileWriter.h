#pragma once

#include <Trajectory.h>
#include <util/logger/logging.hpp>
#include <filems/write/strategies/WriteStrategy.h>

#include <mutex>
#include <string>
#include <sstream>
#include <memory>

namespace helios { namespace filems{

/**
  * @author Alberto M. Esmoris Pena
  * @version 1.0
  * @brief Abstract class defining common behavior for all synchronous file
  *  writers
  * @tparam WriteArgs Arguments for the write operation
  * @see filems::WriteStrategy
  */
template <typename ... WriteArgs>
class SyncFileWriter{
protected:
    /**
     * @brief Path to file to be written
     */
    std::string path;
    /**
     * @brief Mutex to synchronize concurrent write operations
     */
    std::mutex mutex;
    /**
     * @brief The write strategy specifying how to write data to file
     */
    std::shared_ptr<WriteStrategy<WriteArgs ...>> writeStrategy = nullptr;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for synchronous file writer
     */
    SyncFileWriter() = default;

    /**
     * @brief Instantiate a SyncFileWriter which writes to file at given path
     * @param path Path to file to be written
     * @see SyncFileWriter::path
     */
    explicit SyncFileWriter(const std::string & path) : path(path) {};
    virtual ~SyncFileWriter(){}

    // ***  W R I T E  *** //
    // ******************* //
    /**
     * @brief Synchronously write to file
     */
    void write(WriteArgs ... writeArgs){
        // Get the mutex to have exclusive access
        std::lock_guard<std::mutex> lock(mutex);

        // Write data function
        try {
            writeStrategy->write(writeArgs ...);
        }
        catch(std::exception &e){
            std::stringstream ss;
            ss << "SyncFileWriter failed to write. EXCEPTION: \n\t"
                << e.what();
            logging::WARN(ss.str());
        }
    }

    // ***  F I N I S H  *** //
    // ********************* //
    /**
     * @brief Finish the writing so all writing operations are performed and
     * all buffers are closed
     */
    virtual void finish() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the path to the file
     * @return Path to the file
     */
    inline std::string getPath(){return path;}
};

}}
