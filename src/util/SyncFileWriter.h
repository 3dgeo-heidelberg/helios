#pragma once

#include <Measurement.h>
#include <mutex>
#include <string>
#include <Trajectory.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class defining common behavior for all synchronous file
 * writers
 */
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
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
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
     * @brief Synchronously write Measurement to file
     * @param m Measurement to be written
     * @param shift Shift for the measurement position
     * @see Measurement
     */
    void write(
        Measurement const &m,
        glm::dvec3 const shift = glm::dvec3(0, 0, 0)
    ){
        // Get the mutex to have exclusive access
        std::lock_guard<std::mutex> lock(mutex);

        // Write data function
        try {
            _write(m, shift);
        }
        catch(std::exception &e){
            std::stringstream ss;
            ss << "SyncFileWriter failed to write measurement. EXCEPTION: \n\t"
                << e.what();
            logging::WARN(ss.str());
        }
    }
    /**
     * @brief Abstract write function for Measurement. Must be overridden by
     * children classes.
     * @param m Measurement to be written.
     * @param shift Shift for the measurement position
     * @see Measurement
     */
    virtual void _write(Measurement const &m, glm::dvec3 const & shift) = 0;

     /**
      * @brief Synchronously write trajectory to file
      * @param t Trajectory to be written.
      * @see Trajectory
      */
    void write(Trajectory const &t){
        // Get the mutex to have exclusive access
        std::lock_guard<std::mutex> lock(mutex);

        // Write data function
        try{
            _write(t);
        }
        catch(std::exception &e){
            std::stringstream ss;
            ss << "SyncFileWriter failed to write trajectory. EXCEPTION: \n\t"
                << e.what();
            logging::WARN(ss.str());
        }
    }

    /**
     * @brief Abstract write function for trajectory. Must be overridden by
     * children classes.
     */
    virtual void _write(Trajectory const &t) = 0;


    /**
     * @brief Synchronously write Fullwave to file
     * @param fullwave Fullwave data
     * @param fullwaveIndex Fullwave index
     * @param minTime Min hit time for the fullwave (ns)
     * @param maxTime Max hit time for the fullwave (ns)
     * @param beamOrigin Origin for the beam
     * @param beamDir Director vector for the beam
     * @param gpsTime GPS time
     */
    void write(
        std::vector<double> const &fullwave,
        int fullwaveIndex,
        double minTime,
        double maxTime,
        glm::dvec3 const & beamOrigin,
        glm::dvec3 const & beamDir,
        long gpsTime
    ){
        // Get the mutex to have exclusive access
        std::lock_guard<std::mutex> lock(mutex);

        // Write data function
        try {
            _write(
                fullwave,
                fullwaveIndex,
                minTime,
                maxTime,
                beamOrigin,
                beamDir,
                gpsTime
            );
        }
        catch(std::exception &e){
            std::stringstream ss;
            ss << "SyncFileWriter failed to write fullwave. EXCEPTION: \n\t"
               << e.what();
            logging::WARN(ss.str());
        }
    }

    /**
     * @brief Abstract write function for Fullwave. Must be overridden by
     * children classes.
     * @param fullwave Fullwave data
     * @param fullwaveIndex Fullwave index
     * @param minTime Min hit time for the fullwave (ns)
     * @param maxTime Max hit time for the fullwave (ns)
     * @param beamOrigin Origin for the beam
     * @param beamDir Director vector for the beam
     * @param gpsTime GPS time
     */
    virtual void _write(
        std::vector<double> const &fullwave,
        int fullwaveIndex,
        double minTime,
        double maxTime,
        glm::dvec3 const & beamOrigin,
        glm::dvec3 const & beamDir,
        long gpsTime
    ) = 0;

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