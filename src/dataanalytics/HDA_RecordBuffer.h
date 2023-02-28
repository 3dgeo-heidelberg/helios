#ifdef DATA_ANALYTICS
#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <functional>

namespace helios { namespace analytics {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The HeliosDataAnalytics record buffer. It is, a class which handles
 *  the storage of records in memory and its writing to the corresponding file.
 *  Thus, the record buffer is in fact a write buffer.
 * @tparam T The type of element stored at the buffer.
 */
template <typename T>
class HDA_RecordBuffer {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The buffer itself
     */
    std::vector<T> buff;
    /**
     * @brief The maximum number of elements supported by the buffer
     */
    size_t maxSize;
    /**
     * @brief The path to the output file were records are written
     */
    std::string outpath;
    /**
     * @brief The output stream to write the contents of the buffer
     */
    std::ofstream ofs;
    /**
     * @brief The separator between recorded elements
     */
    std::string sep;
    /**
     * @brief The function to write the content of the buffer through the
     *  output stream
     */
    std::function<void(void)> write;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a HDA_RecordBuffer to handle the storage and writing of
     *  records.
     * @see HDA_RecordBuffer::outpath
     * @see HDA_RecordBuffer::maxSize
     */
    HDA_RecordBuffer(
        std::string const &outpath,
        size_t const maxSize=256,
        std::string const &sep=","
    ) :
        maxSize(maxSize),
        outpath(outpath),
        ofs(outpath, std::ios_base::out),
        sep(sep)
    {
        write = [&](void) -> void {this->firstWrite();};
    }

    virtual ~HDA_RecordBuffer(){
        if(isOpen()) close();
    }

    // ***  RECORD BUFFER METHODS  *** //
    // ******************************* //
    /**
     * @brief Check whether the output stream of the record buffer is opened or
     *  not
     * @return True if the output stream is opened, false otherwise
     */
    inline bool isOpen() {return ofs.is_open();}
    /**
     * @brief Write the contents of the buffer through the output stream for
     *  the first time
     */
    inline void firstWrite() {
        size_t numElems = buff.size();
        ofs << buff[0];
        for(size_t i = 1 ; i < numElems ; ++i){
            ofs << sep << buff[i];
        }
        this->write = [&] (void) -> void {this->nextWrite();};
    }
    /**
     * @brief Write the contents of the buffer through the output stream after
     *  the first time
     */
    inline void nextWrite() {
        for(T const & elem : buff) ofs << sep << elem;
    }
    /**
     * @brief Write all the contents of the buffer and then make it empty
     */
    inline void flush(){
        if(buff.size() > 0){
            this->write();
            buff.clear();
        }
    }
    /**
     * @brief Write all the contents of the buffer, make it empty and close the
     *  output stream
     */
    inline void close(){
        flush();
        if(isOpen()) ofs.close();
    }
    /**
     * @brief Insert given element in the buffer. If the buffer is full, it
     *  will be automatically flushed before inserting the new element.
     * @param elem The element to be inserted
     * @see HDA_RecordBuffer::flush
     */
    inline void push(T const & elem){
        if(buff.size() >= maxSize) flush();
        buff.push_back(elem);
    }


};

}}

#endif