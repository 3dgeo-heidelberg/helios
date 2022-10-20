#pragma once

#include <filems/write/comps/SyncFileWriter.h>
#include <filems/write/strategies/WriteStrategy.h>

#include <mutex>
#include <memory>

namespace helios { namespace filems{


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class defining common behavior for all synchronous writers
 *  that work with multiple files at a time
 * @see filems::WriteStrategy
 * @see filems::SyncFileWriter
 */
template <typename ... WriteArgs>
class MultiSyncFileWriter : public SyncFileWriter<WriteArgs ...>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Paths to the files to be written
     */
    std::vector<std::string> path;
    /**
     * @brief One mutex per file to be written (i-th mutex corresponds to
     *  i-th file)
     */
    std::vector<std::mutex> mutex;
    /**
     * @brief The write strategies specifying how to write data, one per file.
     */
    std::vector<std::shared_ptr<WriteStrategy<WriteArgs ...>>> writeStrategy;

    // *** CONSTRUCTION / DESTRUCTION  *** //
    // *********************************** //
    /**
     * @brief Default constructor for synchronous multi-file writer
     * @param path Paths to the files to be written
     * @see MultiSyncFileWriter::path
     */
    MultiSyncFileWriter() : SyncFileWriter<WriteArgs ...>() {}
    explicit MultiSyncFileWriter(std::vector<std::string> const &path) :
        SyncFileWriter<WriteArgs ...>(),
        path(path)
    {}
    virtual ~MultiSyncFileWriter(){}

    // ***  W R I T E  *** //
    // ******************* //
    /**
     * @brief Synchronously write to a single file
     * @see SyncFileWriter::write
     */
    void write(WriteArgs ... writeArgs) override {
        // Obtain the index of file to be written from the write arguments
        size_t const idx = indexFromWriteArgs(writeArgs ...);

        // Get the mutex to have exclusive access
        std::lock_guard<std::mutex> lock(mutex[idx]);

        // Write data function
        try {
            writeStrategy[idx]->write(writeArgs ...);
        }
        catch(std::exception &e){
            std::stringstream ss;
            ss << "MultiSyncFileWriter failed to write. EXCEPTION: \n\t"
               << e.what();
            logging::WARN(ss.str());
        }
    }
    /**
     * @brief Any concrete implementation extending MultiSyncFileWriter must
     *  override this function to specify how to obtain the index from the
     *  WriteArgs
     * @param writeArgs The arguments defining a write operation
     * @return The index corresponding to the requested write operation
     */
    virtual size_t indexFromWriteArgs(WriteArgs ... writeArgs) = 0;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the path to the idx-th file
     * @param idx The index of the file which path must be obtained
     * @return Path to the idx-th file
     */
    std::string getPath(size_t const idx) const override {return path[idx];}
};


}}
