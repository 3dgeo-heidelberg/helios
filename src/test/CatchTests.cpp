#include <catch2/catch_test_macros.hpp>
#include "logging.hpp"

bool    logging::LOGGING_SHOW_TRACE,    logging::LOGGING_SHOW_DEBUG,
        logging::LOGGING_SHOW_INFO,     logging::LOGGING_SHOW_TIME,
        logging::LOGGING_SHOW_WARN,     logging::LOGGING_SHOW_ERR;

TEST_CASE( "Test test", "[]" ) {
    REQUIRE( 1==1 );
}