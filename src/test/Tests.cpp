#include <boost/filesystem.hpp>
#include <test/BaseTest.h>
#include <test/RandomTest.h>
#include <test/NoiseTest.h>
#include <test/VoxelParsingTest.h>
#include <test/RayIntersectionTest.h>
#include <test/SerializationTest.h>
#include <test/SurveyCopyTest.h>
#include <test/PlaneFitterTest.h>
#include <test/LadLutTest.h>
#include <test/PlatformPhysicsTest.h>
#include <test/ScenePartSplitTest.h>

#if defined(_WIN32) || defined(_WIN64)
#define TEST_COLOR false
#else
#define TEST_COLOR true
#endif

using namespace HeliosTests;

void doTests(){
    // Validate current working directory
    if(boost::filesystem::current_path().leaf() != "helios-plusplus"){
        std::cout <<
        "WARNING! You might not be inside helios-plusplus root directory.\n\t"
        "This might cause tests to fail." << std::endl;
    }


    // ***  T E S T S  *** //
    // ******************* //
    RandomTest randomTest;
    randomTest.test(std::cout, TEST_COLOR);

    NoiseTest noiseTest;
    noiseTest.test(std::cout, TEST_COLOR);

    VoxelParsingTest voxelParsingTest;
    voxelParsingTest.test(std::cout, TEST_COLOR);

    RayIntersectionTest rayIntersectionTest;
    rayIntersectionTest.test(std::cout, TEST_COLOR);

    SerializationTest serializationTest;
    serializationTest.test(std::cout, TEST_COLOR);

    SurveyCopyTest surveyCopyTest;
    surveyCopyTest.test(std::cout, TEST_COLOR);

    PlaneFitterTest planeFitterTest;
    planeFitterTest.test(std::cout, TEST_COLOR);

    LadLutTest ladLutTest;
    ladLutTest.test(std::cout, TEST_COLOR);

    PlatformPhysicsTest platformPhysicsTest;
    platformPhysicsTest.test(std::cout, TEST_COLOR);

    ScenePartSplitTest scenePartSplitTest;
    scenePartSplitTest.test(std::cout, TEST_COLOR);
}