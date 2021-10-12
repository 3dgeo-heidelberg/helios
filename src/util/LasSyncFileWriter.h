#pragma once

#include <SyncFileWriter.h>
#include <mutex>
#include <string>
#include <ctime>
#include <laswriter.hpp>
#include <helios_version.h>
#include <glm/glm.hpp>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief SyncFileWriter implementation for LAS format
 */
class LasSyncFileWriter : public SyncFileWriter{
public:
    // ***  CONSTANTS  *** //
    // ******************* //
    /**
     * @brief Classification mask constant for LAS format
     */
    static const U8 CLASSIFICATION_MASK = 31;
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief LASwriter opener. Used to instantiate LASwriter lw.
     */
    LASwriteOpener lwOpener;
    /**
     * @brief LASwriter. Used to write to LAS file.
     */
    std::shared_ptr<LASwriter> lw;
    /**
     * @brief Header definition for the LAS file.
     */
    LASheader lwHeader;
    /**
     * @brief LASpoint used to build different points which shall be
     * written to LAS output file.
     */
    LASpoint lp;
    /**
     * @brief Scale factor for coordinates
     *
     * \f[
     *  X_{coord} = \left( {X_{record} \cdot X_{scale}} \right) + X_{offset}
     * \f]
     */
    double scaleFactor;
    /**
     * @brief Inverse of the scale factor
     *
     * \f[
     *  \frac{1}{\textrm{scaleFactor}}
     * \f]
     */
    double scaleFactorInverse;
    /**
     * @brief Offset for coordinates
     *
     * \f[
     *  X_{coord} = \left( {X_{record} \cdot X_{scale}} \right) + X_{offset}
     * \f]
     */
    glm::dvec3 offset;
    /**
     * @brief Minimum value for intensity. Values less than this will be
     * clipped to minIntensity.
     */
    double minIntensity;
    /**
     * @brief Maximum value for intensity. Values greater than this will be
     * clipped to maxIntensity.
     */
    double maxIntensity;
    /**
     * @brief The difference between max and min intensity.
     *
     * \f[
     *  \Delta_{I} = I_{max} - I_{min}
     * \f]
     */
    double deltaIntensity;
    /**
     * @brief Flag used to control the sync writer status
     */
    bool finished;
    /**
     * @brief Precomputed intensity coefficient
     *
     * \f[
     *  I_{c} = \frac{65535}{\Delta_{I}}
     * \f]
     *
     * \f$65535\f$ comes from \f$2^{16} - 1\f$ as LAS standard defines 16 bits
     * to store intensity
     */
    double intensityCoefficient;
    /**
     * @brief Index of echo width attribute in LAS header definition
     */
    I32 ewAttrIdx;
    /**
     * @brief Index of full wave index attribute in LAS header definition
     */
    I32 fwiAttrIdx;
    /**
     * @brief Index of hit object ID attribute in LAS header definition
     */
    I32 hoiAttrIdx;
    /**
     * @brief Index of helios amplitude attribute in LAS header definition
     */
    I32 ampAttrIdx;
    /**
     * @brief Echo width attribute start (LAS extra bytes format)
     */
    I32 ewAttrStart;
    /**
     * @brief Full wave index attribute start (LAS extra bytes format)
     */
    I32 fwiAttrStart;
    /**
     * @brief Hit object ID attribute start (LAS extra bytes format)
     */
    I32 hoiAttrStart;
    /**
     * @brief Helios amplitude attribute start (LAS extra bytes format)
     */
    I32 ampAttrStart;
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    LasSyncFileWriter() : SyncFileWriter() {};
    /**
     * @brief Synchronous LAS file writer constructor
     * @see SyncFileWriter::path
     * @param compress Specify is use compressed LAS format (LAZ) or not (pure
     *  LAS)
     * @see LasSyncFileWriter::scaleFactor
     * @see LasSyncFileWriter::offset
     * @see LasSyncFileWriter::minIntensity
     * @see LasSyncFileWriter::deltaIntensity
     */
    explicit LasSyncFileWriter(
        const std::string &path,
        bool compress = false,
        double scaleFactor = 0.0001,
        glm::dvec3 offset = glm::dvec3(0, 0, 0),
        double minIntensity = 0.0,
        double deltaIntensity = 1000000.0

    ) :
        SyncFileWriter(path),
        scaleFactor(scaleFactor),
        offset(offset),
        minIntensity(minIntensity),
        deltaIntensity(deltaIntensity),
        finished(false)
    {
        // Craft header and point format
        craft();

        // Add extra attributes
        addExtraAttributes();

        // Create LASWriter
        createLasWriter(path, compress);

    }

    ~LasSyncFileWriter() override {LasSyncFileWriter::finish();}

    // *** CRAFTING *** //
    /**
     * @brief Crafting of header of the LAS File for version 1.0
     */
    void craft()
    {
      // Prepare
      std::stringstream ss;
      scaleFactorInverse = 1.0 / scaleFactor;
      maxIntensity = minIntensity + deltaIntensity;
      intensityCoefficient = 65535.0 / deltaIntensity;


      // Build header
      ss << "HELIOS++V" << HELIOS_VERSION;
      std::string ssStr = ss.str();
      size_t n = ssStr.length();
      if(n > 31) n = 31;
      memcpy(
          lwHeader.generating_software,
          ssStr.c_str(),
          sizeof(char) * n
      );
      lwHeader.generating_software[n] = 0;
      ss.str("");
      lwHeader.version_major = U8(1);
      lwHeader.version_minor = U8(0); // las version 1.0
      time_t now = time(0);
      tm *gmtm = gmtime(&now);
      lwHeader.file_creation_day = gmtm->tm_yday;
      lwHeader.file_creation_year = 1900 + gmtm->tm_year;
      lwHeader.point_data_format = 1;
      lwHeader.point_data_record_length = 28;
      lwHeader.x_scale_factor = scaleFactor;
      lwHeader.y_scale_factor = scaleFactor;
      lwHeader.z_scale_factor = scaleFactor;
      lwHeader.x_offset = F64(offset.x);
      lwHeader.y_offset = F64(offset.y);
      lwHeader.z_offset = F64(offset.z);
      lwHeader.global_encoding = 0;
    }

    // *** EXTRA ATTRIBUTES *** //
    /**
     * @brief Creation of extra attributes to be added to each record
     */
    void addExtraAttributes()
    {
      // Extra bytes
      ewAttrIdx = -1;
      fwiAttrIdx = -1;
      hoiAttrIdx = -1;
      ampAttrIdx = -1;
      try{
        I32 ewType = 9;  // double
        I32 fwiType = 5; // int
        I32 hoiType = 5; // int
        I32 ampType = 9; // double
        LASattribute ewAttr(
            ewType,
            "echo_width",
            "Helios++ echo width"
        );
        LASattribute fwiAttr(
            fwiType,
            "fullwaveIndex",
            "Helios++ fullwave index"
        );
        LASattribute hoiAttr(
            hoiType,
            "hitObjectId",
            "Helios++ hit object ID"
        );
        LASattribute ampAttr(
            ampType,
            "heliosAmplitude",
            "Helios++ Amplitude"
        );
        ewAttrIdx = lwHeader.add_attribute(ewAttr);
        fwiAttrIdx = lwHeader.add_attribute(fwiAttr);
        hoiAttrIdx = lwHeader.add_attribute(hoiAttr);
        ampAttrIdx = lwHeader.add_attribute(ampAttr);
      }
      catch(std::exception &e){
        std::stringstream ss;
        ss  << "LasSyncFileWriter failed.\n\tEXCEPTION: "
            << e.what();
        logging::WARN(ss.str());
      }

      lwHeader.update_extra_bytes_vlr();
      lwHeader.point_data_record_length += lwHeader.get_attributes_size();
      ewAttrStart = lwHeader.get_attribute_start(ewAttrIdx);
      fwiAttrStart = lwHeader.get_attribute_start(fwiAttrIdx);
      hoiAttrStart = lwHeader.get_attribute_start(hoiAttrIdx);
      ampAttrStart = lwHeader.get_attribute_start(ampAttrIdx);
    }

    /**
     * @brief Creation of the LasWriter itself, including LASpoint
     * initialization
     * @param path Path where the file will be save
     * @param compress Flag to activate/deactivate compression (las/laz format)
     */
    void createLasWriter(const std::string & path, bool compress)
    {
      // Initialize LASpoint
      lp.init(
          &lwHeader,
          lwHeader.point_data_format,
          lwHeader.point_data_record_length,
          0
      );

      /* Create the writer itself */
      lwOpener.set_file_name(path.c_str());
      if(compress) lwOpener.set_format(LAS_TOOLS_FORMAT_LAZ);
      else lwOpener.set_format(LAS_TOOLS_FORMAT_LAS);
      lw = std::shared_ptr<LASwriter>(lwOpener.open(&lwHeader));
    }

    // ***  W R I T E  *** //
    // ******************* //
    /**
     * @brief Write measurement to LAS file
     * @see SyncFileWriter::_write(Measurement const &, glm::dvec3 const &)
     */
    void _write(Measurement const &m, glm::dvec3 const & shift) override{
        // Compute shifted position
        glm::dvec3 shifted = m.position + shift - offset;

        // Populate LAS point (base attributes)
        lp.set_X(I32(shifted.x * scaleFactorInverse));
        lp.set_Y(I32(shifted.y * scaleFactorInverse));
        lp.set_Z(I32(shifted.z * scaleFactorInverse));
        double intensityf = m.intensity;
        if(intensityf < minIntensity) intensityf = minIntensity;
        if(intensityf > maxIntensity) intensityf = maxIntensity;
        U16 intensity = U16(
            intensityCoefficient * (intensityf - minIntensity)
        );
        lp.set_intensity(intensity);

        lp.set_return_number(U8(m.returnNumber));
        lp.set_extended_return_number(U8(m.returnNumber));

        lp.set_number_of_returns(U8(m.pulseReturnNumber));
        lp.set_extended_number_of_returns(U8(m.pulseReturnNumber));

        lp.set_classification(U8(m.classification) & CLASSIFICATION_MASK);
        lp.set_extended_classification(U8(m.classification) & CLASSIFICATION_MASK);

        lp.set_gps_time(F64(((double)m.gpsTime)/1000.0));

        // Populate LAS point (extra bytes attributes)
        lp.set_attribute(ewAttrStart, F64(m.echo_width));
        lp.set_attribute(fwiAttrStart, I32(m.fullwaveIndex));
        lp.set_attribute(hoiAttrStart, I32(std::stoi(m.hitObjectId)));
        lp.set_attribute(ampAttrStart, F64(m.intensity));

        // Write and add to inventory
        lw->write_point(&lp);
        lw->update_inventory(&lp);
    }

    /**
     * @brief Write fullwave to LAS file
     * @see SyncFileWriter::_write(std::vector
     */
    void _write(
        std::vector<double> const &fullwave,
        int fullwaveIndex,
        double minTime,
        double maxTime,
        glm::dvec3 const & beamOrigin,
        glm::dvec3 const & beamDir,
        long gpsTime
    ) override {
        throw HeliosException(
            "LasSyncFileWriter does NOT support fullwave writing"
        );
    }

    /**
     * @brief Write trajectory to compressed file
     * @see SimpleSyncFileWriter::_write(
     *  long,
     *  double, double, double,
     *  double, double, double
     * )
     */
    void _write(Trajectory const &t) override {
        throw HeliosException(
            "LasSyncFileWriter does NOT support trajectory writing"
        );
    }

    // ***  F I N I S H  *** //
    // ********************* //
    /**
     * @brief LasSyncFileWriter updates header and guarantees writings
     * have been done only after the finish method has been invoked.
     * If it has not been manually invoked, then it will when destroying the
     * instance.
     * Once the finish method has been invoked, the LasSyncFileWriter should
     * not be used again.
     */
    void finish() override{
        if(finished) return;

        lw->update_header(&lwHeader, true);

        lwHeader.remove_attribute(ewAttrIdx);
        lwHeader.remove_attribute(fwiAttrIdx);
        lwHeader.remove_attribute(hoiAttrIdx);
        lwHeader.remove_attribute(ampAttrIdx);
        free(lwHeader.attributes);
        free(lwHeader.attribute_starts);
        free(lwHeader.attribute_sizes);

        lw->close();

        finished = true;
    };
};