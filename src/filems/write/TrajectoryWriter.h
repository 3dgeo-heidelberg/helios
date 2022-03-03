#pragma once

#include <filems/write/HeliosWriter.h>
#include <filems/write/SimpleSyncFileWriter.h>
#include <scanner/Trajectory.h>

#include <boost/filesystem.hpp>

#include <memory>

namespace fs=boost::filesystem;

namespace helios { namespace filems{

using std::shared_ptr;
using std::make_shared;
using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle writing of trajectories to generate HELIOS++ output
 *  virtual trajectories
 */
class TrajectoryWriter : public HeliosWriter{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for trajectory writer
     */
    TrajectoryWriter() = default;
    virtual ~TrajectoryWriter() = default;

    // ***   M E T H O D S   *** //
    // ************************* //
    /**
     * @brief Configure the output path for the trajectory writer
     * @param parent Path to output directory for trajectory files
     * @param prefix Prefix for the name of the output file
     */
    void configure(string const &parent, string const &prefix);
    /**
     * @brief Write a trajectory point
     * @param t Trajectory point to be written
     */
    void writeTrajectory(Trajectory & t);

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Set synchronous file writer for trajectory writer
     * @param sfw Synchronous file writer to be used to write trajectory
     * @see filems::TrajectoryWriter::sfw
     */
    inline void setSyncFileWriter(shared_ptr<SyncFileWriter> sfw){
        this->sfw = sfw;
    }
    /**
     * @brief Get the synchronous file writer used to write trajectories
     * @see filems::TrajectoryWriter::sfw
     */
    inline shared_ptr<SyncFileWriter> getSyncFileWriter() const {return sfw;}
    /**
     * @brief Set path to output file
     * @param path New path to output file
     */
    inline void setOutputFilePath(string const &path)
    {setSyncFileWriter(make_shared<SimpleSyncFileWriter>(path));}
    /**
     * @brief Get the path to the output file
     * @return The path to the output file
     * @see filems::TrajectoryWriter::getOutputPath
     */
    inline fs::path getOutputFilePath() const
    {return fs::path(getOutputPath());}
    /**
     * @see filems::TrajectoryWriter::getOutputFilePath
     */
    inline string getOutputPath() const {return sfw->getPath();}


};

}}