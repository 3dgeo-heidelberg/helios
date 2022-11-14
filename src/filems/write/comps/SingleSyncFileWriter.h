#pragma once

#include <filems/write/comps/SyncFileWriter.h>
#include <filems/write/strategies/WriteStrategy.h>

#include <mutex>
#include <memory>

namespace helios { namespace filems{


/**
 * @author Alberto M. Esmoris PEna
 * @version 1.0
 * @brief Abstract class defining common behavior for all synchronous writers
 *  that work with a single file at a time
 * @see filems::WriteStrategy
 * @see filems::SyncFileWriter
 */
template <typename ... WriteArgs>
class SingleSyncFileWriter : public SyncFileWriter<WriteArgs ...>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
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


    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for synchronous single-file writer
     */
    SingleSyncFileWriter() : SyncFileWriter<WriteArgs ...>() {}

    /**
     * @brief Instantiate a SingleSyncFileWriter which writes to file at
     *  given path
     * @param path Path to file to be written
     * @see SingleSyncFileWriter::path
     */
    explicit SingleSyncFileWriter(const std::string & path) :
        SyncFileWriter<WriteArgs ...>(),
        path(path)
    {}
    virtual ~SingleSyncFileWriter(){}

    // ***  W R I T E  *** //
    // ******************* //
    /**
     * @brief Synchronously write to a single file
     * @see SyncFileWriter::write
     */
    void write(WriteArgs ... writeArgs) override {
        // Get the mutex to have exclusive access
        std::lock_guard<std::mutex> lock(mutex);

        // Write data function
        try {
            writeStrategy->write(writeArgs ...);
        }
        catch(std::exception &e){
            std::stringstream ss;
            ss << "SingleSyncFileWriter failed to write. EXCEPTION: \n\t"
               << e.what();
            logging::WARN(ss.str());
        }
    }

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the path to the file
     * @return Path to the file
     */
    std::string getPath(size_t const idx) const override {return path;}
};

}}