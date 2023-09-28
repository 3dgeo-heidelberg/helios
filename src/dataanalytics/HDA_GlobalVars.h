#if DATA_ANALYTICS >=2
#pragma once
#include <cstdlib>
#include <mutex>

namespace helios { namespace analytics{

// ***  GLOBAL OBJECT  *** //
// *********************** //
extern class HDA_GlobalVars HDA_GV;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class to handle global variables together with methods for safe
 *  readings and updates.
 *
 * The HDA_GlobalVars class provides a series of variables and corresponding
 *  methods for safe operations. For example, variables that must be
 *  accessed through parallel threads are handled with a mutex such that, as
 *  long as the proper methods are used, read and write operations are
 *  thread safe.
 */
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
};

}}

#endif
