#include <SwapOnRepeatHandler.h>
#include <AbstractGeometryFilter.h>
#include <XYZPointCloudFileLoader.h>
#include <ScenePart.h>
#include <scene/primitives/Primitive.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SwapOnRepeatHandler::SwapOnRepeatHandler() :
    currentTimeToLive(1),
    baseline(nullptr),
    discardOnReplay(false),
    holistic(false),
    onSwapFirstPlay(false)
{}


// ***   MAIN METHODS   *** //
// ************************ //
void SwapOnRepeatHandler::swap(ScenePart &sp){
    --currentTimeToLive;
    if(currentTimeToLive < 1) doSwap(sp);
}

void SwapOnRepeatHandler::prepare(ScenePart * sp){
    numTargetSwaps = (int) swapFilters.size();
    std::deque<int> ttls(timesToLive);
    numTargetReplays = 0;
    while(!ttls.empty()){
        numTargetReplays += ttls.front();
        ttls.pop_front();
    }
    numCurrentSwaps = 0;
    this->baseline = std::make_unique<ScenePart>(*sp);
    for(Primitive *p : this->baseline->mPrimitives) p->part = nullptr;
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void SwapOnRepeatHandler::pushSwapFilters(
    std::deque<AbstractGeometryFilter *> const &swapFilters
){
    this->swapFilters.push_back(swapFilters);
}

void SwapOnRepeatHandler::pushTimeToLive(int const timeToLive){
    this->timesToLive.push_back(timeToLive);
}

std::vector<Primitive *> & SwapOnRepeatHandler::getBaselinePrimitives(){
    return baseline->mPrimitives;
}

// ***  UTIL METHODS  *** //
// ********************** //
void SwapOnRepeatHandler::doSwap(ScenePart &sp){
    // Get next queue of filters and update time to live
    std::deque<AbstractGeometryFilter *> filters = swapFilters.front();
    swapFilters.pop_front();
    currentTimeToLive = timesToLive.front();
    timesToLive.pop_front();

    // Apply filters
    bool firstIter = true;
    while(!filters.empty()){
        // Get next filter
        AbstractGeometryFilter *filter = filters.front();
        filters.pop_front();
        // Run the filter
        ScenePart * genSP = filter->run();
        // Update the geometry if a new one has been loaded
        if(genSP != nullptr && genSP != std::addressof(sp)){
            // Make holistic only if geometry is derived from a point cloud
            holistic = false;
            if(dynamic_cast<XYZPointCloudFileLoader *>(filter) != nullptr){
                holistic = true;
            }
            doGeometricSwap(*genSP, sp);
            // Delete generated geometry (it will no longer be used)
            delete genSP;
        }
        // Otherwise
        else{
            // Reload the baseline geometry on the first iteration only
            if(firstIter) {
                // Backup primitives to prevent that baseline is updated
                std::vector<Primitive *> primitivesBackup = baseline->mPrimitives;
                for(size_t i = 0 ; i < primitivesBackup.size() ; ++i){
                    Primitive * newPrimitive = primitivesBackup[i]->clone();
                    newPrimitive->part = nullptr;
                    baseline->mPrimitives[i] = newPrimitive;
                }
                // TODO Rethink : Solves the memory leak? ---
                // Free primitives memory from scene part
                for(Primitive *p : sp.mPrimitives) delete p;
                // --- TODO Rethink : Solves the memory leak?
                // The geometric swap itself
                doGeometricSwap(*baseline, sp);
                // Restore baseline to prevent any update to propagate further
                baseline->mPrimitives = primitivesBackup;
            }
            // Remove pointer to primitives to prevent delete current ones
        }
        // Delete filter
        filter->primsOut = nullptr;
        delete filter;
        firstIter = false;
    }

    // Update current swaps count and activate first play flag
    ++numCurrentSwaps;
    onSwapFirstPlay = true;
}

void SwapOnRepeatHandler::doGeometricSwap(ScenePart &src, ScenePart &dst){
    // Assign to dst from src
    dst.primitiveType = src.primitiveType;
    dst.mPrimitives = src.mPrimitives;
    dst.centroid = src.centroid;
    dst.bound = src.bound;
    dst.onRayIntersectionMode = src.onRayIntersectionMode;
    dst.onRayIntersectionArgument = src.onRayIntersectionArgument;
    dst.randomShift = src.randomShift;
    dst.ladlut = src.ladlut;
    dst.mCrs = src.mCrs;
    dst.mEnv = src.mEnv;
}
