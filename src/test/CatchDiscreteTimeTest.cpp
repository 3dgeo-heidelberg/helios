#include <sim/tools/DiscreteTime.h>
#include <catch2/catch_test_macros.hpp>

TEST_CASE( "Discrete time test", "[]" ) {
    // setup
    DiscreteTime dt1(500, 1.0);

    SECTION("toDiscrete") {
        auto [input, expected] = GENERATE(std::make_tuple(0.5, 250), std::make_tuple(1.5, 750));
        REQUIRE(dt1.toDiscrete(input) == expected);
    }

    SECTION("toCyclicDiscrete and toPeriodicDiscrete") {
        auto i = GENERATE(0.5, 1.5, 3.5);
        
        SECTION("toCyclicDiscrete") {
            REQUIRE( dt1.toCyclicDiscrete(i) == 250 );
        }
        
        SECTION("toPeriodicDiscrete") {
            REQUIRE( dt1.toPeriodicDiscrete(i) == 250 );
        }
    }

}

TEST_CASE( "Discrete time test 2", "[]" ) {

}