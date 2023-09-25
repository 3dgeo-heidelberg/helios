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

        // Extract variables for the sake of convenience
        size_t const raycasterLeafFailedTminCheckCount =
            HDA_GV.getRaycasterLeafFailedTminCheckCount();
        size_t const raycasterLeafFailedTmaxCheckCount =
            HDA_GV.getRaycasterLeafFailedTmaxCheckCount();
        size_t const subrayLeafFailedTminCheckCount =
            HDA_GV.getSubrayLeafFailedTminCheckCount();
        size_t const subrayLeafFailedTmaxCheckCount =
            HDA_GV.getSubrayLeafFailedTmaxCheckCount();

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
        ss  << "\tNon-intersective subrays due to null time: "
            << gv.getNonIntersectiveSubraysDueToNullTimeCount() << "\n";
        ss  << "Subray intersections: "
            << gv.getSubrayIntersectionCount() << "\n";
        ss  << "Subray non-intersections: "
            << gv.getSubrayNonIntersectionCount() << "\n";
        ss  << "Number of computed intensities: "
            << gv.getIntensityComputationsCount() << "\n";
        ss  << "Raycaster fails-on-leaves distribution:\n"
            << "\tNegative distances: "
            << HDA_GV.getRaycasterLeafNegativeDistancesCount() << "\n"
            << "\tFurther than current closest: "
            << HDA_GV.getRaycasterLeafFurtherThanClosestCount() << "\n"
            << "\tFailed tmin checks: "
            << raycasterLeafFailedTminCheckCount << "\n"
            << "\tFailed tmax checks: "
            << raycasterLeafFailedTmaxCheckCount << "\n"
            << "\tFailed t checks: "
            << (
                    raycasterLeafFailedTminCheckCount +
                    raycasterLeafFailedTmaxCheckCount
               ) << "\n"
            << "\tSubrays only:\n"
            << "\t\tNegative distances: "
            << HDA_GV.getSubrayLeafNegativeDistancesCount() << "\n"
            << "\t\tFurther than current closest: "
            << HDA_GV.getSubrayLeafFurtherThanClosestCount() << "\n"
            << "\t\tFailed tmin checks: "
            << subrayLeafFailedTminCheckCount << "\n"
            << "\t\tFailed tmax checks: "
            << subrayLeafFailedTmaxCheckCount << "\n"
            << "\t\tFailed t checks: "
            << (
                    raycasterLeafFailedTminCheckCount +
                    raycasterLeafFailedTmaxCheckCount
            ) << "\n";
        // Print through info logging level system
        std::string text = ss.str();
        logging::INFO(ss.str());
    }
};

}}

#endif
