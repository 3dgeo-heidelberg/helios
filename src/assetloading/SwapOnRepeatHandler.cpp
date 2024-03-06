#include <SwapOnRepeatHandler.h>
#include <AbstractGeometryFilter.h>
#include <ScenePart.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SwapOnRepeatHandler::SwapOnRepeatHandler() :
    baseline(nullptr)
{}


// ***   MAIN METHODS   *** //
// ************************ //
void SwapOnRepeatHandler::swap(ScenePart &sp){
    // Get next queue of filters
    std::deque<AbstractGeometryFilter *> filters = swapFilters.back();
    swapFilters.pop_back();

    // Apply filters
    // TODO Rethink : Rewrite s.t. while(! filters.empty)
    bool firstIter = true;
    while(!filters.empty()){
        // Get next filter
        AbstractGeometryFilter *filter = filters.back();
        filters.pop_back();
        // Run the filter
        ScenePart * genSP = filter->run();
        // Update the geometry if a new one has been loaded
        if(genSP != nullptr && genSP != filter->primsOut){
            doGeometricSwap(*genSP, sp);
            // Delete generated geometry (it will no longer be used)
            delete genSP;
        }
        // Otherwise
        else{
            // Reload the baseline geometry on the first iteration only
            if(firstIter) {
                doGeometricSwap(*baseline, sp);
            }
            // Remove pointer to primitives to prevent delete current ones
            filter->primsOut = nullptr;
        }
        // Delete filter
        delete filter;
        firstIter = false;
    }

    // Update current swaps count
    ++numCurrentSwaps;
}

void SwapOnRepeatHandler::prepare(std::shared_ptr<ScenePart> sp){
    numTargetSwaps = (int) swapFilters.size();
    numCurrentSwaps = 0;
    this->baseline = std::make_unique<ScenePart>(*sp);
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void SwapOnRepeatHandler::pushSwapFilters(
    std::deque<AbstractGeometryFilter *> const &swapFilters
){
    this->swapFilters.push_back(swapFilters);
}

// ***  UTIL METHODS  *** //
// ********************** //
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
    // TODO Rethink : Delete old primitives from dst before updating them?
}
