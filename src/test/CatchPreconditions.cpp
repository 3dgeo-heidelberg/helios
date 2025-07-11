#include <catch2/catch_test_macros.hpp>

#include <boost/filesystem.hpp>
#include <sstream>

TEST_CASE("Catch Testing Preconditions") {
    // This test is used to check that the test executable is correctly executed from the root directory.
    // Validate required test files can be located
    std::vector<std::string> EXPECTED_FILES({ "semitransparent_voxels.vox", "spherical.txt" });
    std::string testDir = "data/test";
    for (std::string& expectedFile : EXPECTED_FILES) {
        std::stringstream ss;
        char const pathsep = boost::filesystem::path::preferred_separator;
        ss << testDir << pathsep << expectedFile;
        INFO("WARNING! You might not be inside helios root directory.\n\t"
            "This will cause tests to fail.\n"
            "Please, change to helios root directory.\n"
            "If this warning persists then probably test data "
            "files cannot be located.\n"
            "Get sure files can be located and accessed in your system.\n"
            "If the problem persists, please report it."
        );
        REQUIRE(boost::filesystem::exists(ss.str()));
    }
}