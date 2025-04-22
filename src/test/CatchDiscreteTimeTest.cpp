#include <sim/tools/DiscreteTime.h>
#include <catch2/catch_test_macros.hpp>

TEST_CASE( "Discrete time test", "[]" ) {
    DiscreteTime dt1(500, 1.0);

    SECTION("toDiscrete and toContinuous") {
        auto [cont, disc] = GENERATE(std::make_tuple(0.5, 250), std::make_tuple(1.5, 750));
        
        SECTION("toDiscrete") {
            REQUIRE(dt1.toDiscrete(cont) == disc);
        }
        
        SECTION("toContinuous") {
            REQUIRE(dt1.toContinuous(disc) == cont)
        }
    }

    SECTION("toCyclicDiscrete and toPeriodicDiscrete") {
        auto i = GENERATE(0.5, 1.5, 3.5);
        
        SECTION("toCyclicDiscrete") {
            REQUIRE(dt1.toCyclicDiscrete(i) == 250);
        }
        
        SECTION("toPeriodicDiscrete") {
            REQUIRE(dt1.toPeriodicDiscrete(i) == 250);
        }
    }

    SECTION("toCyclicContinuous and toPeriodicContinuous") {
        auto i = GENERATE(250, 750, 1750);

        SECTION("toCyclicContinuous") {
            REQUIRE(dt1.toCyclicContinuous(i) == 0.5);
        }

        SECTION("toPeriodicContinuous") {
            REQUIRE(dt1.toPeriodicContinuous(i) == 0.5);
        }
    }

}

TEST_CASE( "Discrete time test 2", "[]" ) {
    DiscreteTime dt2(500, 2.0);

    SECTION("toDiscrete and toContinuous") {
        auto [cont, disc] = GENERATE(std::make_tuple(0.5, 250), std::make_tuple(1.5, 750));
        
        SECTION("toDiscrete") {
            REQUIRE(dt2.toDiscrete(cont) == disc);
        }
        
        SECTION("toContinuous") {
            REQUIRE(dt2.toContinuous(disc) == cont)
        }
    }

    SECTION("toCyclicDiscrete") {
        auto i = GENERATE(0.5, 1.5, 3.5);

        REQUIRE(dt2.toCyclicDiscrete(i) == 250);
    }

    SECTION("toPeriodicDiscrete") {
        auto [cont, disc] = GENERATE(std::make_tuple(0.0, 0), std::make_tuple(0.2, 100), std::make_tuple(0.5, 250), std::make_tuple(1.5, 750), std::make_tuple(2.0, 0), std::make_tuple(2.2, 100), std::make_tuple(2.5, 250), std::make_tuple(3.5, 750));

        REQUIRE(dt2.toPeriodicDiscrete(cont) == disc);
    }

    SECTION("toCyclicContinuous") {
        auto i = GENERATE(250, 750, 1750);

        REQUIRE(dt2.toCyclicContinuous(i) == 0.5);
    }

    SECTION("toPeriodicContinuous") {
        auto [disc, cont] = GENERATE(std::make_tuple(0, 0.0), std::make_tuple(100, 0.2), std::make_tuple(250, 0.5), std::make_tuple(500, 1.0), std::make_tuple(750, 1.5), std::make_tuple(1000, 0.0), std::make_tuple(1250, 0.5), std::make_tuple(1500, 1.0), std::make_tuple(1750, 1.5));

        REQUIRE(dt2.toPeriodicContinuous(disc) == cont);
    }
}