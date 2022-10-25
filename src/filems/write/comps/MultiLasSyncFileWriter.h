#pragma once

#include <filems/write/comps/MultiSyncFileWriter.h>
#include <filems/util/LasWriterSpec.h>

#include <laswriter.hpp>

#include <vector>
#include <memory>

namespace helios { namespace filems{

using std::vector;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract specialization of MultiSyncFileWriter to write output in
 *  LAS format
 * @see filems::MultiSyncFileWriter
 */
template <typename ... WriteArgs>
class MultiLasSyncFileWriter : public MultiSyncFileWriter<WriteArgs ...>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The specifications defining each LAS writer
     */
    vector<LasWriterSpec> lws;
    /**
     * @brief The LASwriter used to write to each LAS file.
     */
    vector<shared_ptr<LASwriter>> lw;
    /**
     * @brief Flag used to control the sync writer status
     */
    bool finished;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for Synchronous Multi LAS file writer
     */
    MultiLasSyncFileWriter() : MultiSyncFileWriter<WriteArgs...>() {};
    explicit MultiLasSyncFileWriter(
        vector<string> const &path,
        bool const compress,
        vector<double> const &scaleFactor,
        vector<glm::dvec3> const &offset,
        vector<double> const &minIntensity,
        vector<double> const &deltaIntensity,
        bool const createWriters = true
    ) :
        MultiSyncFileWriter<WriteArgs ...>(path),
        finished(false)
    {
        // Initialize the specification for eachj LAS writer
        size_t nWriters = path.size();
        for(size_t i = 0 ; i < nWriters ; ++i){
            lws.push_back(LasWriterSpec(
                path[i],
                scaleFactor[i],
                offset[i],
                minIntensity[i],
                deltaIntensity[i]
            ));
        }
        // If construct must create the writers
        if(createWriters){
            // Create each LASWriter
            createLasWriters(path, compress);
        }
    }
    virtual ~MultiLasSyncFileWriter() {MultiLasSyncFileWriter::finish();}

    // ***  CREATE WRITER  *** //
    // *********************** //
    /**
     * @brief Creation of each LasWriter , including LASpoint
     * initialization
     * @param path Path for each file
     * @param compress Flag to activate/deactivate compression (las/laz format)
     */
    void createLasWriters(vector<string> const &path, bool const compress)
    {
        size_t const nWriters = path.size();
        for(size_t i = 0 ; i < nWriters ; ++i){ // For each i-th writer
            // Extract path and writer spec
            string const &path = this->path[i];
            LasWriterSpec &lws = this->lws[i];
            // Craft header and point format
            lws.craft();
            // Add extra attributes
            lws.addExtraAttributes();
            // Initialize LASpoint
            lws.initLASPoint();
            // Create writer from specification
            lw.push_back(lws.makeWriter(path, compress));
        }
    }

    // ***  F I N I S H  *** //
    // ********************* //
    /**
     * @brief MultiLasSyncFileWriter updates each header and guarantees all
     *  writings have been done only after the finish method has been invoked.
     * If it has not been manually invoked, then it will when destroying the
     *  instance.
     * Once the finish method has been invoked, the MultiLasSyncFileWriter
     *  should not be used again.
     * @see filems::LasWriterSpec::finish
     */
    void finish() override{
        if(finished) return; // Check whether finished or not
        // Finish each writer and its specification
        size_t const nWriters = lw.size();
        for(size_t i = 0 ; i < nWriters ; ++i){
            // Extract writer and specification
            shared_ptr<LASwriter> lw = this->lw[i];
            LasWriterSpec &lws = this->lws[i];
            // Do finishing stuff
            lw->update_header(&lws.lwHeader, true); // Update the header
            lws.finish(); // Finish the initialized specification
            lw->close(); // Close the writer itself
        }
        // Flag the MultiLasSyncFileWriter as finished
        finished = true;
    }
};

}}