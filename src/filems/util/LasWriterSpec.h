#pragma once

#include <helios_version.h>

#include <laswriter.hpp>
#include <glm/glm.hpp>

#include <string>
#include <sstream>

namespace helios { namespace filems{

using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing the specification defining a LasWriter
 *  (not the writer itself)
 * @see filems::LasSyncFileWriter
 * @see filems::MultiLasSyncFileWriter
 */
class LasWriterSpec{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief LASwriter opener. Used to instantiate LASwriter lw.
     */
    LASwriteOpener lwOpener;
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

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for LasWriterSpec
     */
    LasWriterSpec() = default;
    /**
     * @brief LAS writer specification constructor
     * @param compress Specify if use compressed LAS format (LAZ) or not (pure
     *  LAS)
     * @see LasWriterSpec::scaleFactor
     * @see LasWriterSpec::ofset
     * @see LasWriterSpec::minIntensity
     * @see LasWriterSpec::deltaIntensity
     */
    explicit LasWriterSpec(
        const string &path,
        double const scaleFactor = 0.0001,
        glm::dvec3 const offset = glm::dvec3(0, 0, 0),
        double const minIntensity = 0.0,
        double const deltaIntensity = 1000000.0
    ) :
        scaleFactor(scaleFactor),
        offset(offset),
        minIntensity(minIntensity),
        deltaIntensity(deltaIntensity)
    {}
    /**
     * @brief Copy constructor. It must be overridden to prevent segmentation
     *  fault when copying the LAS attributes such as the LASpoint
     * @see LasWriterSpec::lp
     */
    LasWriterSpec(LasWriterSpec const &lws){
        scaleFactor = lws.scaleFactor;
        scaleFactorInverse = lws.scaleFactorInverse;
        offset = lws.offset;
        minIntensity = lws.minIntensity;
        maxIntensity = lws.maxIntensity;
        deltaIntensity = lws.deltaIntensity;
        intensityCoefficient = lws.intensityCoefficient;
        ewAttrIdx = lws.ewAttrIdx;
        fwiAttrIdx = lws.fwiAttrIdx;
        hoiAttrIdx = lws.hoiAttrIdx;
        ampAttrIdx = lws.ampAttrIdx;
        ewAttrStart = lws.ewAttrStart;
        fwiAttrStart = lws.fwiAttrStart;
        hoiAttrStart = lws.hoiAttrStart;
        ampAttrStart = lws.ampAttrStart;
    }
    virtual ~LasWriterSpec() = default;

    // ***   C R A F T I N G   *** //
    // *************************** //
    /**
     * @brief Craft the header of the LAS File for version 1.0
     */
    void craft(){
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
    /**
     * @brief Craft the header of the LAS File for version 1.4
     */
    void craft14(){
        // Craft LAS 1.0 version
        craft();

        // Update Header to 1.4 specification
        lwHeader.version_minor = U8(4);

        // Update Point Data Format to support new return number / classes
        lwHeader.point_data_format = 0;
        lwHeader.point_data_record_length = 30;

        // Adds the byte difference between LAS 1.4 and LAS 1.0 (350 - 227)
        lwHeader.header_size += 148;

        // Adds the byte difference to the data point offset
        lwHeader.offset_to_point_data += 148;
    }

    // ***  EXTRA ATTRIBUTES  *** //
    // ************************** //
    /**
     * @brief Creation of extra attributes to be added to each record
     */
    void addExtraAttributes(){
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

    // ***  INIT LAS POINT  *** //
    // ************************ //
    /**
     * @brief Initialize the LAS point structure with data from header
     */
    void initLASPoint(){
        lp.init(
            &lwHeader,
            lwHeader.point_data_format,
            lwHeader.point_data_record_length,
            0
        );
    }

    // ***  MAKE WRITER  *** //
    // ********************* //
    /**
     * @brief Build a LAS writer from this specification
     * @return Built LAS writer from current state of specification
     */
    shared_ptr<LASwriter> makeWriter(
        std::string const &path, bool const compress
    ){
        // Create LAS writer from specification
        lwOpener.set_file_name(path.c_str());
        if(compress) lwOpener.set_format(LAS_TOOLS_FORMAT_LAZ);
        else lwOpener.set_format(LAS_TOOLS_FORMAT_LAS);
        return std::shared_ptr<LASwriter>(lwOpener.open(&lwHeader));
    }

    // ***  F I N I S H   *** //
    // ********************** //
    /**
     * @brief Remove and release everything that has been initialized in the
     *  process of building the writer from the specification
     * @see filems::LasSyncFileWriter::finish
     */
    void finish(){
        lwHeader.remove_attribute(ewAttrIdx);
        lwHeader.remove_attribute(fwiAttrIdx);
        lwHeader.remove_attribute(hoiAttrIdx);
        lwHeader.remove_attribute(ampAttrIdx);
        free(lwHeader.attributes);
        free(lwHeader.attribute_starts);
        free(lwHeader.attribute_sizes);
    }

};


}}
