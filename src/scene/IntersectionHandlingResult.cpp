#include "IntersectionHandlingResult.h"

#include <sstream>

// ***  STREAM OPERATORS  *** //
// ************************** //
std::ostream& operator<< (
    std::ostream &out,
    IntersectionHandlingResult const &ihr
){
    std::stringstream ss;
    ss  << "Intersection Handling Result:\n"
        << "\tIntersectionPoint = (" << ihr.intersectionPoint[0] << ", " <<
        ihr.intersectionPoint[1] << ", " << ihr.intersectionPoint[2] << ")\n"
        << "\tcanContinue = " << ihr.canContinue << "\n";
    out << ss.str();
    return out;
}
