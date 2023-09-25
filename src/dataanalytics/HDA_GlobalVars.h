#ifdef DATA_ANALYTICS
#pragma once
#include <cstdlib>
#include <mutex>

namespace helios { namespace analytics{

// ***  GLOBAL OBJECT  *** //
// *********************** //
extern class HDA_GlobalVars HDA_GV;

// TODO Rethink : Document
class HDA_GlobalVars{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    std::size_t generatedRaysBeforeEarlyAbortCount;
    std::size_t generatedRaysAfterEarlyAbortCount;
    std::size_t generatedSubraysCount;
    std::size_t intersectiveSubraysCount;
    std::size_t nonIntersectiveSubraysCount;
    std::size_t nonIntersectiveSubraysDueToNullTimeCount;
    std::size_t subrayIntersectionCount;
    std::size_t subrayNonIntersectionCount;
    std::size_t intensityComputationsCount;
    std::size_t raycasterLeafNegativeDistancesCount;
    std::size_t raycasterLeafFurtherThanClosestCount;
    std::size_t raycasterLeafFailedTminCheckCount;
    std::size_t raycasterLeafFailedTmaxCheckCount;
    unsigned long subrayLeafNegativeDistancesCount;
    unsigned long subrayLeafFurtherThanClosestCount;
    unsigned long subrayLeafFailedTminCheckCount;
    unsigned long subrayLeafFailedTmaxCheckCount;

protected:
    // ***  CONCURRENCY HANDLING ATTRIBUTES  *** //
    // ***************************************** //
    std::mutex generatedRaysBeforeEarlyAbortCount_mutex;
    std::mutex generatedRaysAfterEarlyAbortCount_mutex;
    std::mutex generatedSubraysCount_mutex;
    std::mutex intersectiveSubraysCount_mutex;
    std::mutex nonIntersectiveSubraysCount_mutex;
    std::mutex nonIntersectiveSubraysDueToNullTimeCount_mutex;
    std::mutex subrayIntersectionCount_mutex;
    std::mutex subrayNonIntersectionCount_mutex;
    std::mutex intensityComputationsCount_mutex;
    std::mutex raycasterLeafNegativeDistancesCount_mutex;
    std::mutex raycasterLeafFurtherThanClosestCount_mutex;
    std::mutex raycasterLeafFailedTminCheckCount_mutex;
    std::mutex raycasterLeafFailedTmaxCheckCount_mutex;
    std::mutex subrayLeafNegativeDistancesCount_mutex;
    std::mutex subrayLeafFurtherThanClosestCount_mutex;
    std::mutex subrayLeafFailedTminCheckCount_mutex;
    std::mutex subrayLeafFailedTmaxCheckCount_mutex;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    HDA_GlobalVars() :
        generatedRaysBeforeEarlyAbortCount(0),
        generatedRaysAfterEarlyAbortCount(0),
        generatedSubraysCount(0),
        intersectiveSubraysCount(0),
        nonIntersectiveSubraysCount(0),
        nonIntersectiveSubraysDueToNullTimeCount(0),
        subrayIntersectionCount(0),
        subrayNonIntersectionCount(0),
        intensityComputationsCount(0)
    {}
    virtual ~HDA_GlobalVars() = default;

    // ***  WRITE METHODS  *** //
    // *********************** //
    HDA_GlobalVars & incrementGeneratedRaysBeforeEarlyAbortCount();
    HDA_GlobalVars & incrementGeneratedRaysAfterEarlyAbortCount();
    HDA_GlobalVars & incrementGeneratedSubraysCount();
    HDA_GlobalVars & incrementIntersectiveSubraysCount();
    HDA_GlobalVars & incrementNonIntersectiveSubraysCount();
    HDA_GlobalVars & incrementNonIntersectiveSubraysDueToNullTimeCount();
    HDA_GlobalVars & incrementSubrayIntersectionCount();
    HDA_GlobalVars & incrementSubrayNonIntersectionCount();
    HDA_GlobalVars & incrementIntensityComputationsCount();
    HDA_GlobalVars & incrementRaycasterLeafNegativeDistancesCount();
    HDA_GlobalVars & incrementRaycasterLeafFurtherThanClosestCount();
    HDA_GlobalVars & incrementRaycasterLeafFailedTminCheckCount();
    HDA_GlobalVars & incrementRaycasterLeafFailedTmaxCheckCount();
    HDA_GlobalVars & incrementSubrayLeafNegativeDistancesCount();
    HDA_GlobalVars & incrementSubrayLeafFurtherThanClosestCount();
    HDA_GlobalVars & incrementSubrayLeafFailedTminCheckCount();
    HDA_GlobalVars & incrementSubrayLeafFailedTmaxCheckCount();

    // ***  READ METHODS  *** //
    // ********************** //
    std::size_t getGeneratedSubraysCount();
    std::size_t getGeneratedRaysBeforeEarlyAbortCount();
    std::size_t getGeneratedRaysAfterEarlyAbortCount();
    std::size_t getIntersectiveSubraysCount();
    std::size_t getNonIntersectiveSubraysCount();
    std::size_t getNonIntersectiveSubraysDueToNullTimeCount();
    std::size_t getSubrayIntersectionCount();
    std::size_t getSubrayNonIntersectionCount();
    std::size_t getIntensityComputationsCount();
    std::size_t getRaycasterLeafNegativeDistancesCount();
    std::size_t getRaycasterLeafFurtherThanClosestCount();
    std::size_t getRaycasterLeafFailedTminCheckCount();
    std::size_t getRaycasterLeafFailedTmaxCheckCount();
    unsigned long getSubrayLeafNegativeDistancesCount();
    unsigned long getSubrayLeafFurtherThanClosestCount();
    unsigned long getSubrayLeafFailedTminCheckCount();
    unsigned long getSubrayLeafFailedTmaxCheckCount();
};

}}

#endif
