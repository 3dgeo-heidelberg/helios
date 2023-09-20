#ifdef DATA_ANALYTICS
#pragma once
#include <HDA_GlobalVars.h>
#include <util/logger/logging.hpp>

#include <sstream>
#include <string>

namespace helios { namespace analytics{

// TODO Rethink : Document
class HDA_GlobalVarsReporter{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    HDA_GlobalVars & gv;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    explicit HDA_GlobalVarsReporter(HDA_GlobalVars &gv) : gv(gv) {}
    virtual ~HDA_GlobalVarsReporter() = default;

    // ***   PRINT   *** //
    // ***************** //
    void print(){
        // Initialize string stream to build the print
        std::stringstream ss;

        // Print global variables
        ss << "HDA GLOBAL VARS REPORT:\n\n";
        ss  << "Generated rays before early abort: "
            << gv.getGeneratedRaysBeforeEarlyAbortCount() << "\n";
        ss  << "Generated rays after early abort: "
            << gv.getGeneratedRaysAfterEarlyAbortCount() << "\n";
        ss << "Generated subrays: " << gv.getGeneratedSubraysCount() << "\n";
        ss  << "Intersective subrays: "
            << gv.getIntersectiveSubraysCount() << "\n";
        ss  << "Non-intersective subrays: "
            << gv.getNonIntersectiveSubraysCount() << "\n";
        ss  << "Subray intersections: "
            << gv.getSubrayIntersectionCount() << "\n";
        ss  << "Subray non-intersections: "
            << gv.getSubrayNonIntersectionCount() << "\n";
        ss  << "Number of computed intensities: "
            << gv.getIntensityComputationsCount() << "\n";
        // Print through info logging level system
        std::string text = ss.str();
        logging::INFO(ss.str());
    }
};

}}

#endif
