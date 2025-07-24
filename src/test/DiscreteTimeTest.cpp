#include <sim/tools/DiscreteTime.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

TEST_CASE( "Discrete time test" ) {
    SECTION("Discrete time test with f=500, periodScale=1.0") {

        DiscreteTime dt1(500, 1.0);

        SECTION("toDiscrete and toContinuous") {
            auto [cont, disc] = GENERATE(std::make_tuple(0.5, 250), std::make_tuple(1.5, 750));
            
            REQUIRE(dt1.toDiscrete(cont) == disc);
            REQUIRE(dt1.toContinuous(disc) == cont);
        }

        SECTION("toCyclicDiscrete and toPeriodicDiscrete") {
            auto i = GENERATE(0.5, 1.5, 3.5);
            
            REQUIRE(dt1.toCyclicDiscrete(i) == 250);
            REQUIRE(dt1.toPeriodicDiscrete(i) == 250);
        }

        SECTION("toCyclicContinuous and toPeriodicContinuous") {
            auto i = GENERATE(250, 750, 1750);

            REQUIRE(dt1.toCyclicContinuous(i) == 0.5);
            REQUIRE(dt1.toPeriodicContinuous(i) == 0.5);
        }

        SECTION("toDiscrete and toContinuous") {
            REQUIRE(dt1.toDiscrete(0.00001) == 0);
            REQUIRE(dt1.toContinuous(0) == 0.0);
            REQUIRE(dt1.toContinuous(1) > 0.0);
        }
    }

    SECTION( "Discrete time test with f=500, periodScale=2.0" ) {
        DiscreteTime dt2(500, 2.0);

        SECTION("toDiscrete and toContinuous") {
            auto [cont, disc] = GENERATE(std::make_tuple(0.5, 250), std::make_tuple(1.5, 750));
            
            REQUIRE(dt2.toDiscrete(cont) == disc);
            REQUIRE(dt2.toContinuous(disc) == cont);
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

        SECTION("Precision") {
            REQUIRE(dt2.toCyclicDiscrete(0.00001) == 0);
            REQUIRE(dt2.toPeriodicDiscrete(0.00001) == 0);
            REQUIRE(dt2.toDiscrete(0.99999999) == 499);
            REQUIRE(dt2.toPeriodicDiscrete(1.99999999) == 999);
            REQUIRE(std::fabs(dt2.toPeriodicContinuous(999)-1.9999) > 0.0001);
        }
    }

    SECTION( "Discrete time test with f=500, periodScale=0.5" ) {
        DiscreteTime dt3(500, 0.5);
        
        SECTION("toDiscrete and toContinuous") {
            auto [cont, disc] = GENERATE(std::make_tuple(0.5, 250), std::make_tuple(1.5, 750));
            
            REQUIRE(dt3.toDiscrete(cont) == disc);
            REQUIRE(dt3.toContinuous(disc) == cont);
        }

        SECTION("toCyclicDiscrete") {
            auto i = GENERATE(0.5, 1.5, 3.5);

            REQUIRE(dt3.toCyclicDiscrete(i) == 250);
        }

        SECTION("toPeriodicDiscrete") {
            auto [cont, disc] = GENERATE(std::make_tuple(0.0, 0), std::make_tuple(0.2, 100), std::make_tuple(0.5, 0), std::make_tuple(1.5, 0), std::make_tuple(2.0, 0), std::make_tuple(2.2, 100), std::make_tuple(2.5, 0), std::make_tuple(3.5, 0));

            REQUIRE(dt3.toPeriodicDiscrete(cont) == disc);
        }

        SECTION("toCyclicContinuous") {
            auto [disc, cont] = GENERATE(std::make_tuple(250, 0.5), std::make_tuple(750, 0.5), std::make_tuple(1750, 0.5));

            REQUIRE(dt3.toCyclicContinuous(disc) == cont);
        }

        SECTION("toPeriodicContinuous") {
            auto [disc, cont] = GENERATE(std::make_tuple(0, 0.0), std::make_tuple(100, 0.2), std::make_tuple(250, 0.0), std::make_tuple(500, 0.0), std::make_tuple(750, 0.0), std::make_tuple(1000, 0.0), std::make_tuple(1250, 0.0), std::make_tuple(1350, 0.2), std::make_tuple(1500, 0.0), std::make_tuple(1750, 0.0));
            
            REQUIRE(dt3.toPeriodicContinuous(disc) == cont);
        }

        SECTION("Trivial cases and precision") {
            REQUIRE(dt3.toCyclicContinuous(0) == 0.0);
            REQUIRE(dt3.toPeriodicContinuous(1) > 0.0);
            REQUIRE(std::fabs(dt3.toContinuous(499)-0.999999) > 0.000001);
            REQUIRE(dt3.toPeriodicDiscrete(0.49999999) == 249);
            REQUIRE(std::fabs(dt3.toPeriodicContinuous(249)-0.4999) > 0.0001);
        }
    }

    SECTION( "Discrete time test with f=1000000, periodScale=1.0" ) {
        DiscreteTime dt4(1000000, 1.0);

        SECTION("toDiscrete") {
            auto [cont, disc] = GENERATE(std::make_tuple(0.00000000001, 0), std::make_tuple(0.000001, 1), std::make_tuple(0.999999999999, 999999), std::make_tuple(3.00000000001, 3000000), std::make_tuple(2.000001, 2000001), std::make_tuple(4.999999999999, 4999999));

            REQUIRE(dt4.toDiscrete(cont) == disc);
        }

        SECTION("toCyclicDiscrete and toPeriodicDiscrete") {
            auto [cont, disc] = GENERATE(std::make_tuple(3.00000000001, 0), std::make_tuple(2.000001, 1), std::make_tuple(4.999999999999, 999999));
            
            REQUIRE(dt4.toCyclicDiscrete(cont) == disc);
            REQUIRE(dt4.toPeriodicDiscrete(cont) == disc);
        }

        SECTION("toContinuous") {
            auto [disc, cont] = GENERATE(std::make_tuple(0, 0.0), std::make_tuple(1, 0.000001));

            REQUIRE(dt4.toContinuous(disc) == cont);
        }

        SECTION("toContinuous with many digits and differences") {
            auto [disc, subtrahend, cont] = GENERATE(std::make_tuple(999999, 0.999999, 0.0000001), std::make_tuple(2000001, 2.000001, 0.0000001), std::make_tuple(4999999, 4.999999, 0.0000001));

            REQUIRE(dt4.toContinuous(disc) - subtrahend < cont);
        }

        SECTION("toContinuous with absolute") {
            auto [disc, cont] = GENERATE(std::tuple(3000000, 3.0));

            REQUIRE(std::fabs(dt4.toContinuous(disc)) == cont);
        }

        SECTION("toCyclicContinuous and toPeriodicContinuous with absolute") {
            auto [disc, cont] = GENERATE(std::tuple(3000000, 0.0));

            REQUIRE(std::fabs(dt4.toCyclicContinuous(disc)) == cont);
            REQUIRE(std::fabs(dt4.toPeriodicContinuous(disc)) == cont);
        }

        SECTION("toCyclicContinuous and toPeriodicContinuous with many digits and differences") {
            auto [disc, subtrahend, cont] = GENERATE(std::make_tuple(2000001, 0.000001, 0.0000001), std::make_tuple(4999999, 0.999999, 0.0000001));	

            REQUIRE(std::fabs(dt4.toCyclicContinuous(disc) - subtrahend) < cont);
            REQUIRE(std::fabs(dt4.toPeriodicContinuous(disc) - subtrahend) < cont);
        }

        SECTION("Inverse tests with toDiscrete/toContinuous") {
            auto i = GENERATE(0, 500000, 999999, 1000000);

            REQUIRE(dt4.toDiscrete(dt4.toContinuous(i)) == i);
        }

        SECTION("Inverse tests with toCyclicDiscrete/toContinuous and toPeriodicDiscrete/toContinuous") {
            auto [in, exp] = GENERATE(std::make_tuple(1000000, 0));

            REQUIRE(dt4.toCyclicDiscrete(dt4.toContinuous(in)) == exp);
            REQUIRE(dt4.toPeriodicDiscrete(dt4.toContinuous(in)) == exp);
        }

        SECTION("Inverse tests with toContinuous/toDiscrete") {
            auto i = GENERATE(0.0, 0.5, 1.0);

            REQUIRE(dt4.toContinuous(dt4.toDiscrete(i)) == i);
        }

        SECTION("Inverse tests with toCyclicContinuous/toDiscrete and toPeriodicContinuous/toDiscrete") {
            auto [in, exp] = GENERATE(std::make_tuple(1.0, 0.0));

            REQUIRE(dt4.toCyclicContinuous(dt4.toDiscrete(in)) == exp);
            REQUIRE(dt4.toPeriodicContinuous(dt4.toDiscrete(in)) == exp);
        }
    }

}