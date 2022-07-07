#include <test/BaseTest.h>
#include <test/RandomTest.h>
#include <test/NoiseTest.h>
#include <test/DiscreteTimeTest.h>
#include <test/VoxelParsingTest.h>
#include <test/RayIntersectionTest.h>
#include <test/GroveTest.h>
#include <test/SerializationTest.h>
#include <test/SurveyCopyTest.h>
#include <test/PlaneFitterTest.h>
#include <test/LadLutTest.h>
#include <test/PlatformPhysicsTest.h>
#include <test/FunctionalPlatformTest.h>
#include <test/ScenePartSplitTest.h>
#include <test/RigidMotionTest.h>
#include <test/FluxionumTest.h>
#include <test/HPCTest.h>

#include <boost/filesystem.hpp>


#if defined(_WIN32) || defined(_WIN64)
#define TEST_COLOR false
#else
#define TEST_COLOR true
#endif

using namespace HeliosTests;

/**
 * @brief Validate test mode preconditions have been satisfied
 * @param testDir Path to directory containing test data
 * @return True if preconditions are satisfied, false otherwise
 */
bool validateTestsPrecondition(std::string const & testDir);

void doTests(std::string const & testDir){
    // Validate current working directory
    if(!validateTestsPrecondition(testDir)){
        std::cout <<
        "WARNING! You might not be inside helios root directory.\n\t"
        "This might cause tests to fail.\n"
        "Please, change to helios root directory.\n"
        "Alternatively, specify path to directory containing required test "
        "files with:\n\t --testDir <path>\n\n"
        "If this warning persists then probably test data "
        "files cannot be located.\n"
        "Get sure files can be located and accessed in your system.\n"
        "If the problem persists, please report it\n\n"
        "Current test files directory is: \"" << testDir << "\"\n"
        << std::endl;
    }


    // ***  T E S T S  *** //
    // ******************* //
    bool passed = true;
    RandomTest randomTest;
    passed &= randomTest.test(std::cout, TEST_COLOR);

    NoiseTest noiseTest;
    passed &= noiseTest.test(std::cout, TEST_COLOR);

    DiscreteTimeTest discreteTimeTest;
    passed &= discreteTimeTest.test(std::cout, TEST_COLOR);

    VoxelParsingTest voxelParsingTest(testDir);
    passed &= voxelParsingTest.test(std::cout, TEST_COLOR);

    RayIntersectionTest rayIntersectionTest;
    passed &= rayIntersectionTest.test(std::cout, TEST_COLOR);

    GroveTest groveTest;
    passed &= groveTest.test(std::cout, TEST_COLOR);

    SerializationTest serializationTest;
    passed &= serializationTest.test(std::cout, TEST_COLOR);

    SurveyCopyTest surveyCopyTest;
    passed &= surveyCopyTest.test(std::cout, TEST_COLOR);

    PlaneFitterTest planeFitterTest;
    passed &= planeFitterTest.test(std::cout, TEST_COLOR);

    LadLutTest ladLutTest(testDir);
    passed &= ladLutTest.test(std::cout, TEST_COLOR);

    PlatformPhysicsTest platformPhysicsTest;
    passed &= platformPhysicsTest.test(std::cout, TEST_COLOR);

    FunctionalPlatformTest functionalPlatformTest;
    passed &= functionalPlatformTest.test(std::cout, TEST_COLOR);

    ScenePartSplitTest scenePartSplitTest;
    passed &= scenePartSplitTest.test(std::cout, TEST_COLOR);

    RigidMotionTest rigidMotionTest;
    passed &= rigidMotionTest.test(std::cout, TEST_COLOR);

    FluxionumTest fluxionumTest(testDir);
    passed &= fluxionumTest.test(std::cout, TEST_COLOR);

    HPCTest hpcTest;
    passed &= hpcTest.test(std::cout, TEST_COLOR);

    if(!passed) std::exit(3);
}


bool validateTestsPrecondition(std::string const & testDir){
    // Validate required test files can be located
    std::vector<std::string> EXPECTED_FILES({
        "semitransparent_voxels.vox",
        "spherical.txt"
    });
    for(std::string &expectedFile : EXPECTED_FILES){
        std::stringstream ss;
        char const pathsep = boost::filesystem::path::preferred_separator;
        ss  << testDir
            << pathsep
            << expectedFile;
        if(!boost::filesystem::exists(ss.str())) return false;
    }
    return true;
}
