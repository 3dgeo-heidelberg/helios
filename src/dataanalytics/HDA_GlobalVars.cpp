#ifdef DATA_ANALYTICS
#include <HDA_GlobalVars.h>

namespace helios { namespace analytics{


// ***  GLOBAL OBJECT  *** //
// *********************** //
HDA_GlobalVars HDA_GV;

// ***  WRITE METHODS  *** //
// *********************** //
HDA_GlobalVars & HDA_GlobalVars::incrementGeneratedSubraysCount(){
    std::unique_lock<std::mutex> lock(generatedSubraysCount_mutex);
    ++generatedSubraysCount;
    return *this;
}

HDA_GlobalVars &
        HDA_GlobalVars::incrementGeneratedRaysBeforeEarlyAbortCount(){
    std::unique_lock<std::mutex> lock(
        generatedRaysBeforeEarlyAbortCount_mutex
    );
    ++generatedRaysBeforeEarlyAbortCount;
    return *this;
}

HDA_GlobalVars &
        HDA_GlobalVars::incrementGeneratedRaysAfterEarlyAbortCount(){
    std::unique_lock<std::mutex> lock(
        generatedRaysAfterEarlyAbortCount_mutex
    );
    ++generatedRaysAfterEarlyAbortCount;
    return *this;
}

HDA_GlobalVars &
        HDA_GlobalVars::incrementIntensityComputationsCount() {
    std::unique_lock<std::mutex> lock(intensityComputationsCount_mutex);
    ++intensityComputationsCount;
    return *this;
}

HDA_GlobalVars & HDA_GlobalVars::incrementIntersectiveSubraysCount(){
    std::unique_lock<std::mutex> lock(intersectiveSubraysCount_mutex);
    ++intersectiveSubraysCount;
    return *this;
}

HDA_GlobalVars & HDA_GlobalVars::incrementNonIntersectiveSubraysCount(){
    std::unique_lock<std::mutex> lock(nonIntersectiveSubraysCount_mutex);
    ++nonIntersectiveSubraysCount;
    return *this;
}

HDA_GlobalVars &
HDA_GlobalVars::incrementNonIntersectiveSubraysDueToNullTimeCount(){
    std::unique_lock<std::mutex> lock(
        nonIntersectiveSubraysDueToNullTimeCount_mutex
    );
    ++nonIntersectiveSubraysDueToNullTimeCount;
    return *this;

}

HDA_GlobalVars & HDA_GlobalVars::incrementSubrayIntersectionCount() {
    std::unique_lock<std::mutex> lock(subrayIntersectionCount_mutex);
    ++subrayIntersectionCount;
    return *this;
}

HDA_GlobalVars & HDA_GlobalVars::incrementSubrayNonIntersectionCount() {
    std::unique_lock<std::mutex> lock(subrayNonIntersectionCount_mutex);
    ++subrayNonIntersectionCount;
    return *this;
}

HDA_GlobalVars &
HDA_GlobalVars::incrementRaycasterLeafNegativeDistancesCount(){
    std::unique_lock<std::mutex> lock(
        raycasterLeafNegativeDistancesCount_mutex
    );
    ++raycasterLeafNegativeDistancesCount;
    return *this;
}

HDA_GlobalVars &
HDA_GlobalVars::incrementRaycasterLeafFurtherThanClosestCount(){
    std::unique_lock<std::mutex> lock(
        raycasterLeafFurtherThanClosestCount_mutex
    );
    ++raycasterLeafFurtherThanClosestCount;
    return *this;
}

HDA_GlobalVars & HDA_GlobalVars::incrementRaycasterLeafFailedTminCheckCount(){
    std::unique_lock<std::mutex> lock(raycasterLeafFailedTminCheckCount_mutex);
    ++raycasterLeafFailedTminCheckCount;
    return *this;
}

HDA_GlobalVars & HDA_GlobalVars::incrementRaycasterLeafFailedTmaxCheckCount(){
    std::unique_lock<std::mutex> lock(raycasterLeafFailedTmaxCheckCount_mutex);
    ++raycasterLeafFailedTmaxCheckCount;
    return *this;
}

HDA_GlobalVars &
HDA_GlobalVars::incrementSubrayLeafNegativeDistancesCount(){
    std::unique_lock<std::mutex> lock(subrayLeafNegativeDistancesCount_mutex);
    ++subrayLeafNegativeDistancesCount;
    return *this;
}

HDA_GlobalVars &
HDA_GlobalVars::incrementSubrayLeafFurtherThanClosestCount(){
    std::unique_lock<std::mutex> lock(subrayLeafFurtherThanClosestCount_mutex);
    ++subrayLeafFurtherThanClosestCount;
    return *this;
}

HDA_GlobalVars & HDA_GlobalVars::incrementSubrayLeafFailedTminCheckCount(){
    std::unique_lock<std::mutex> lock(subrayLeafFailedTminCheckCount_mutex);
    ++subrayLeafFailedTminCheckCount;
    return *this;
}

HDA_GlobalVars & HDA_GlobalVars::incrementSubrayLeafFailedTmaxCheckCount(){
    std::unique_lock<std::mutex> lock(subrayLeafFailedTmaxCheckCount_mutex);
    ++subrayLeafFailedTmaxCheckCount;
    return *this;
}

// ***  READ METHODS  *** //
// ********************** //
std::size_t HDA_GlobalVars::getGeneratedSubraysCount(){
    std::unique_lock<std::mutex> lock(generatedSubraysCount_mutex);
    return generatedSubraysCount;
}

std::size_t HDA_GlobalVars::getGeneratedRaysBeforeEarlyAbortCount(){
    std::unique_lock<std::mutex> lock(
        generatedRaysBeforeEarlyAbortCount_mutex
    );
    return generatedRaysBeforeEarlyAbortCount;
}

std::size_t HDA_GlobalVars::getGeneratedRaysAfterEarlyAbortCount(){
    std::unique_lock<std::mutex> lock(
        generatedRaysAfterEarlyAbortCount_mutex
    );
    return generatedRaysAfterEarlyAbortCount;
}

std::size_t HDA_GlobalVars::getIntersectiveSubraysCount() {
    std::unique_lock<std::mutex> lock(intersectiveSubraysCount_mutex);
    return intersectiveSubraysCount;
}

std::size_t HDA_GlobalVars::getNonIntersectiveSubraysCount() {
    std::unique_lock<std::mutex> lock(nonIntersectiveSubraysCount_mutex);
    return nonIntersectiveSubraysCount;
}

std::size_t HDA_GlobalVars::getNonIntersectiveSubraysDueToNullTimeCount(){
    std::unique_lock<std::mutex> lock(
        nonIntersectiveSubraysDueToNullTimeCount_mutex
    );
    return nonIntersectiveSubraysDueToNullTimeCount;
}

std::size_t HDA_GlobalVars::getSubrayIntersectionCount() {
    std::unique_lock<std::mutex> lock(subrayIntersectionCount_mutex);
    return subrayIntersectionCount;
}

std::size_t HDA_GlobalVars::getSubrayNonIntersectionCount() {
    std::unique_lock<std::mutex> lock(subrayNonIntersectionCount_mutex);
    return subrayNonIntersectionCount;
}

std::size_t HDA_GlobalVars::getIntensityComputationsCount(){
    std::unique_lock<std::mutex> lock(intensityComputationsCount_mutex);
    return intensityComputationsCount;
}

std::size_t HDA_GlobalVars::getRaycasterLeafNegativeDistancesCount(){
    std::unique_lock<std::mutex> lock(
        raycasterLeafNegativeDistancesCount_mutex
    );
    return raycasterLeafNegativeDistancesCount;
}

std::size_t HDA_GlobalVars::getRaycasterLeafFurtherThanClosestCount(){
    std::unique_lock<std::mutex> lock(
        raycasterLeafFurtherThanClosestCount_mutex
    );
    return raycasterLeafFurtherThanClosestCount;
}

std::size_t HDA_GlobalVars::getRaycasterLeafFailedTminCheckCount(){
    std::unique_lock<std::mutex> lock(raycasterLeafFailedTminCheckCount_mutex);
    return raycasterLeafFailedTminCheckCount;
}

std::size_t HDA_GlobalVars::getRaycasterLeafFailedTmaxCheckCount(){
    std::unique_lock<std::mutex> lock(raycasterLeafFailedTmaxCheckCount_mutex);
    return raycasterLeafFailedTmaxCheckCount;
}

unsigned long HDA_GlobalVars::getSubrayLeafNegativeDistancesCount(){
    std::unique_lock<std::mutex> lock(subrayLeafNegativeDistancesCount_mutex);
    return subrayLeafNegativeDistancesCount;
}

unsigned long HDA_GlobalVars::getSubrayLeafFurtherThanClosestCount(){
    std::unique_lock<std::mutex> lock(subrayLeafFurtherThanClosestCount_mutex);
    return subrayLeafFurtherThanClosestCount;
}

unsigned long HDA_GlobalVars::getSubrayLeafFailedTminCheckCount(){
    std::unique_lock<std::mutex> lock(subrayLeafFailedTminCheckCount_mutex);
    return subrayLeafFailedTminCheckCount;
}

unsigned long HDA_GlobalVars::getSubrayLeafFailedTmaxCheckCount(){
    std::unique_lock<std::mutex> lock(subrayLeafFailedTmaxCheckCount_mutex);
    return subrayLeafFailedTmaxCheckCount;
}



}}

#endif
