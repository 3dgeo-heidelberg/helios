#pragma once

#include <filems/write/comps/SimpleSyncFileStringWriter.h>
#include <maths/Rotation.h>
class SurveyPlayback;

#include <string>

namespace helios { namespace analytics {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The HeliosDataAnalytics state JSON reporter. It is, a class which
 *  generates a JSON representation of the initial HELIOS++ state. It should
 *  be different for different execution configurations.
 */
class HDA_StateJSONReporter{
public:
    // ***  ENUMERATIONS  *** //
    // ********************** //
    /**
     * @brief Types of supported JSON entries
     */
    enum class EntryType{
        VALUE, // "key": val or "key": "val" if string
        OBJECT, // "key": {...}
        ARRAY  // "key": [...]
    };
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The SurveyPlayback containing the data to be written
     */
    SurveyPlayback *sp;
    /**
     * @brief The writer to be used to write the data from SurveyPlayback (sp)
     *  when calling the report method
     * @see HDA_StateJSONReporter::sp
     * @see HDA_StateJSONReporter::report
     */
    helios::filems::SimpleSyncFileStringWriter writer;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a HDA_StateJSONReporter so the state of given
     *  SurveyPlayback and its components (scanner, platform, scene, ...) is
     *  written to the JSON file at given path
     * @param sp The SurveyPlayback containing the data to be written
     * @param path Path where the JSON file must be written
     */
    HDA_StateJSONReporter(
        SurveyPlayback *sp,
        std::string const &path
    ) :
        sp(sp),
        writer(path, std::ios_base::trunc)
    {}
    virtual ~HDA_StateJSONReporter() = default;

    // ***  MAIN REPORT METHODS  *** //
    // ***************************** //
    /**
     * @brief Make the report effective. It is, write it to the corresponding
     *  file.
     */
    virtual void report();

protected:
    // ***  SECONDARY REPORT METHODS  *** //
    // ********************************** //
    /**
     * @brief Add simulation data to the report
     */
    virtual void reportSimulation();
    /**
     * @brief Add survey data to the report
     */
    virtual void reportSurvey();
    /**
     * @brief Add file management system data to the report
     */
    virtual void reportFilems();
    /**
     * @brief Add platform data to the report
     */
    virtual void reportPlatform();

    // ***  UTIL METHODS  *** //
    // ********************** //
    /**
     * @brief Craft a "key": val entry such that tabulations are inserted to
     *  format the depth level at which the entry belongs to.
     * @param key The key for the entry
     * @param val The value for the entry
     * @param depth The depth level at which the entry belongs to.
     * @param asString If true, then the value will be wrapped by quotes,
     *  otherwise it will be considered exactly as given
     * @param last If true, then no comma will be appended as it is considered
     *  to be the last entry inside its parent context
     * @return Crafted entry as a string
     */
    template <typename ValType>
    std::string craftEntry(
        std::string const &key,
        ValType const &val,
        int const depth=0,
        bool const asString=false,
        bool const last=false
    );
    /**
     * @brief Overload HDA_StateJSONReporter::craftEntry to support glm::dvec3
     */
    std::string craftEntry(
        std::string const &key,
        glm::dvec3 const &u,
        int const depth=0,
        bool const asString=false,
        bool const last=false
    );
    /**
     * @brief Overload HDA_StateJSONReporter::craftEntry to support Rotation
     */
    std::string craftEntry(
        std::string const &key,
        Rotation const &r,
        int const depth=0,
        bool const asString=false,
        bool const last=false
    );
    /**
     * @brief Open an entry with given key. If the entry is requested to be
     *  an object
     * @param key The key for the entry
     * @param depth The depth level at which the entry belongs to.
     * @param entryType Specify the type of the entry being opened
     * @return The opening of the entry as a string
     */
    std::string openEntry(
        std::string const &key,
        int const depth=0,
        EntryType const entryType=EntryType::VALUE
    );
    /**
     * @brief Close a previously opened entry.
     * @param depth The depth level at which the entry belongs to.
     * @param last If true, then no comma will be appended as it is considered
     *  to be the last entry inside its parent context
     * @param entryType Specify the type of the entry being closed
     * @return The closing of the entry as a string
     */
    std::string closeEntry(
        int const depth=0,
        bool const last=false,
        EntryType const entryType=EntryType::VALUE
    );
};

}}