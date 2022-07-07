#include <DateTimeUtils.h>
// #include <iostream>
// #include "logging.hpp"

#include <chrono>
#include <ctime>

using namespace std::chrono;

// ***  DATE and TIME UTILS  *** //
// ***************************** //
long DateTimeUtils::dateTimeStrToSeconds(std::string const str){
    std::tm t;
    t.tm_year = stoi(str.substr(0, 4))-1900;
    t.tm_mon = stoi(str.substr(5, 2))-1;
    t.tm_mday = stoi(str.substr(8, 2));
    t.tm_hour = stoi(str.substr(11, 2));
    t.tm_min = stoi(str.substr(14, 2));
    t.tm_sec = stoi(str.substr(17, 2));
    t.tm_isdst = -1;
    // Debug output:
    // std::stringstream ss;
    // ss  << "Provided GPS start time was \"" << str << "\"\n"
    //     << t.tm_year << t.tm_mon <<t.tm_mday << t.tm_hour << t.tm_min << t.tm_sec << '\n';
    // logging::ERR(ss.str());
    return duration_cast<seconds>(system_clock::from_time_t(std::mktime(&t))\
        .time_since_epoch()).count();
}