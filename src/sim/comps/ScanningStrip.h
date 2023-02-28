#pragma once

#include <Leg.h>
#include <HeliosException.h>

#include <string>
#include <sstream>
#include <unordered_map>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief A scanning strip is a set of legs which belong to the same strip.
 *  Thus, it is an abstract group of legs.
 *
 * One leg must belong to either none or at maximum one scanning strip.
 *
 * A scanning strip is used, for instance,
 *  to export to the same output point cloud and trajectory files
 *  all legs belonging to the same strip.
 *
 * @see Leg
 */
class ScanningStrip{
public:
    // ***  CONSTANTS  *** //
    // ******************* //
    static std::string const NULL_STRIP_ID;
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief String identifying the scanning strip
     */
    std::string stripId;
    /**
     * @brief Map with leg serial id as key and pointer to leg as value
     * @see Leg::serialId
     */
    std::unordered_map<int, Leg*> legs;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for scanning strip
     */
    ScanningStrip(std::string const stripId=NULL_STRIP_ID) :
        stripId(stripId)
    {}
    virtual ~ScanningStrip() {}

    // ***  METHODS  *** //
    // ***************** //
    /**
     * @brief Check whether given serial id belongs to the strip or not
     * @param serialId Serial id of leg which association with strip must be
     *  checked
     * @return True if given serial id belongs to the strip, false otherwise
     */
    inline bool has(int const serialId){
        return legs.find(serialId) != legs.end();
    }

    /**
     * @brief Check whether given leg belongs to the strip or not
     * @param leg Leg which association with strip must be checked
     * @return True if given leg belongs to the strip, false otherwise
     */
    inline bool has(Leg &leg){return has(leg.getSerialId());}
    /**
     * @brief Insert/emplace given leg into the scanning strip
     * @param serialId Serial identifier of the leg to be inserted/emplaced
     * @param leg Leg to be inserted/emplaced
     */
    inline void emplace(int const serialId, Leg *leg){
        legs.emplace(serialId, leg);
    }
    /**
     * @see ScanningStrip::emplace(int const, Leg *)
     */
    inline void emplace(Leg *leg){emplace(leg->getSerialId(), leg);}
    /**
     * @brief Like ScanningStrip::emplace(int const, Leg *) but with security
     *  checks
     * @see ScanningStrip::emplace(int const, Leg *)
     */
    inline void safeEmplace(int const serialId, Leg *leg){
        if(has(serialId)){
            std::stringstream ss;
            ss  << "ScanningStrip::safeEmplace will not emplace leg "
                << serialId << " because a leg with same identifier "
                << "already belongs to the strip";
            throw HeliosException(ss.str());
        }
        emplace(serialId, leg);
    }
    /**
     * @see ScanningStrip::safeEmplace(int const, Leg *)
     */
    inline void safeEmplace(Leg *leg){safeEmplace(leg->getSerialId(), leg);}


    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the strip identifier
     * @return Strip identifier
     */
    inline std::string getStripId() const {return stripId;}
    /**
     * @brief Set the new strip identifier
     * @param stripId New strip identifier
     */
    inline void setStripId(std::string const stripId)
    {this->stripId = stripId;}
    /**
     * @brief Obtain leg with given serial identifier
     * @param serialId Serial identifier of leg to be obtained
     * @return Leg with given serial identifier, nullptr if no leg with given
     *  identifier is contained in strip
     */
    inline Leg * getLeg(int const serialId) {
        return has(serialId) ? legs[serialId] : nullptr;
    }
    /**
     * @brief Checks if all the legs in the strip were processed in order
     * to know if the associated SyncFileWriter can be destroyed.
     * @return True if all legs have been processed. False if at least one leg
     *  has not been processed.
     */
    inline bool isLastLegInStrip() const
    {
      for (const auto & it : legs)
        if (!it.second->wasProcessed)
          return false;
      return true;
    }

};