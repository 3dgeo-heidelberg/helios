#pragma once

#include <scanner/Measurement.h>
#include <filems/facade/FMSWriteFacade.h>

#include <cstdlib>
#include <mutex>

using helios::filems::FMSWriteFacade;

using std::size_t;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Yield a point cloud from measurements so it is written when buffer
 *  size has been reached or, alternatively, when yielder is directly forced
 *  to yield
 */
class PointcloudYielder {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The facade for writing operations
     * @see filems::FMSWriteFacade
     */
    FMSWriteFacade &write;
    /**
     * @brief The number of elements that can be buffered before forcing
     *  yield
     */
    size_t bufferSize;
    /**
     * @brief Where the elements are stored
     */
    vector<Measurement> measurements;
    /**
     * @brief The mutex to handle concurrent push backs to measurements
     *  vector and yielding operation itself
     */
    std::mutex mtx;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for point cloud yielder
     * @see PointcloudYielder::write
     * @see PointcloudYielder::bufferSize
     */
    PointcloudYielder(
        FMSWriteFacade &write,
        size_t bufferSize=256
    ) :
        write(write)
    {
        setBufferSize(bufferSize);
    }


    // ***  YIELD METHODS  *** //
    // *********************** //
    /**
     * @brief Make the yielder flush its measurements so the point cloud
     *  writing is done
     */
    void yield();
    /**
     * @brief Push the measurement into the yielder. The measured could be
     *  simply accumulated or either it could be directly written to file
     *  depending on yielder status at push time
     * @param m Measurement to be pushed into the yielder
     */
    inline void push(Measurement const &m){
        mtx.lock();
        measurements.push_back(m);
        mtx.unlock();
        if(measurements.size() >= bufferSize) yield();
    }

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Set the buffer size of the yielder, effectively
     * @param bufferSize New buffer size for the yielder
     */
    inline void setBufferSize(size_t const bufferSize){
        this->bufferSize = bufferSize;
    }
    /**
     * @brief Obtain the current buffer size of the yielder
     * @return The buffer size of the yielder
     */
    inline size_t getBufferSize() const {return bufferSize;}
};