#include <KDGroveFactory.h>
#include <KDGroveStats.h>
#include <KDGroveSubject.h>
#include <TimeWatcher.h>


using std::shared_ptr;
using std::make_shared;

// ***  K-DIMENSIONAL GROVE FACTORY METHODS  *** //
// ********************************************* //
shared_ptr<KDGrove> KDGroveFactory::makeFromSceneParts(
    vector<shared_ptr<ScenePart>> parts,
    bool const mergeNonMoving,
    bool const safe,
    bool const computeKDGroveStats,
    bool const reportKDGroveStats,
    bool const computeKDTreeStats,
    bool const reportKDTreeStats
){
    if(mergeNonMoving){
        return makeMergeNonMoving(
            parts,
            safe,
            computeKDGroveStats,
            reportKDGroveStats,
            computeKDTreeStats,
            reportKDTreeStats
        );
    }
    return makeFull(
        parts,
        safe,
        computeKDGroveStats,
        reportKDGroveStats,
        computeKDTreeStats,
        reportKDTreeStats
    );
}
// ***  STATISTICS METHODS  *** //
// **************************** //
void KDGroveFactory::handleKDGroveStats(
    shared_ptr<KDGrove> kdgrove,
    vector<double> &buildingTimes,
    vector<int> &treePrimitives
){
    size_t const numTrees = kdgrove->getNumTrees();
    KDGroveStats &stats = *kdgrove->getStats();
    stats.numTrees = (int) numTrees; // Number of trees
    for(size_t i = 0 ; i < numTrees ; ++i){
        KDTreeNodeRoot *root = (KDTreeNodeRoot *)
            kdgrove->getTreePointer(i)->root.get();
        // Count dynamic and static trees
        if(root->isDynamic()) ++(stats.numDynTrees);
        else ++(stats.numStaticTrees);
        // Handle min/max statistics
        if(i==0){ // Init value for min/max
            stats.minBuildingTime = buildingTimes[i];
            stats.maxBuildingTime = buildingTimes[i];
            stats.minTreePrimitives = treePrimitives[i];
            stats.maxTreePrimitives = treePrimitives[i];
            stats.minMaxPrimsInLeaf = root->stats_maxNumPrimsInLeaf;
            stats.maxMaxPrimsInLeaf = root->stats_maxNumPrimsInLeaf;
            stats.minMinPrimsInLeaf = root->stats_minNumPrimsInLeaf;
            stats.maxMinPrimsInLeaf = root->stats_minNumPrimsInLeaf;
            stats.minMaxDepth = root->stats_maxDepthReached;
            stats.maxMaxDepth = root->stats_maxDepthReached;
            stats.minSurfaceArea = root->surfaceArea;
            stats.maxSurfaceArea = root->surfaceArea;
            stats.minNumInterior = root->stats_numInterior;
            stats.maxNumInterior = root->stats_numInterior;
            stats.minNumLeaves = root->stats_numLeaves;
            stats.maxNumLeaves = root->stats_numLeaves;
            stats.minCost = root->stats_totalCost;
            stats.maxCost = root->stats_totalCost;
        }
        else{ // Update value for min/max
            stats.minBuildingTime = std::min<double>(
                stats.minBuildingTime, buildingTimes[i]
            );
            stats.maxBuildingTime = std::max<double>(
                stats.maxBuildingTime, buildingTimes[i]
            );
            stats.minTreePrimitives = std::min<int>(
                stats.minTreePrimitives, treePrimitives[i]
            );
            stats.maxTreePrimitives = std::max<int>(
                stats.maxTreePrimitives, treePrimitives[i]
            );
            stats.minMaxPrimsInLeaf = std::min<int>(
                stats.minMaxPrimsInLeaf, root->stats_maxNumPrimsInLeaf
            );
            stats.maxMaxPrimsInLeaf = std::max<int>(
                stats.maxMaxPrimsInLeaf, root->stats_maxNumPrimsInLeaf
            );
            stats.minMinPrimsInLeaf = std::min<int>(
                stats.minMinPrimsInLeaf, root->stats_minNumPrimsInLeaf
            );
            stats.maxMinPrimsInLeaf = std::max<int>(
                stats.maxMinPrimsInLeaf, root->stats_minNumPrimsInLeaf
            );
            stats.minMaxDepth = std::min<int>(
                stats.minMaxDepth, root->stats_maxDepthReached
            );
            stats.maxMaxDepth = std::max<int>(
                stats.maxMaxDepth, root->stats_maxDepthReached
            );
            stats.minSurfaceArea = std::min<double>(
                stats.minSurfaceArea, root->surfaceArea
            );
            stats.maxSurfaceArea = std::max<double>(
                stats.maxSurfaceArea, root->surfaceArea
            );
            stats.minNumInterior = std::min<int>(
                stats.minNumInterior, root->stats_numInterior
            );
            stats.maxNumInterior = std::max<int>(
                stats.maxNumInterior, root->stats_numInterior
            );
            stats.minNumLeaves = std::min<int>(
                stats.minNumLeaves, root->stats_numLeaves
            );
            stats.maxNumLeaves = std::max<int>(
                stats.maxNumLeaves, root->stats_numLeaves
            );
            stats.minCost = std::min<double>(
                stats.minCost, root->stats_totalCost
            );
            stats.maxCost = std::max<double>(
                stats.maxCost, root->stats_totalCost
            );
        }
        // Handle total statistics
        stats.totalBuildingTime += buildingTimes[i];
        stats.totalTreePrimitives += treePrimitives[i];
        stats.totalMaxPrimsInLeaf += root->stats_maxNumPrimsInLeaf;
        stats.totalMinPrimsInLeaf += root->stats_minNumPrimsInLeaf;
        stats.totalMaxDepth += root->stats_maxDepthReached;
        stats.totalSurfaceArea += root->surfaceArea;
        stats.totalNumInterior += root->stats_numInterior;
        stats.totalNumLeaves += root->stats_numLeaves;
        stats.totalCost += root->stats_totalCost;
    }
    // Handle mean statistics
    double const n = (double) stats.numTrees;
    stats.meanBuildingTime = stats.totalBuildingTime / n;
    stats.meanTreePrimitives = stats.totalTreePrimitives / n;
    stats.meanMaxPrimsInLeaf = stats.totalMaxPrimsInLeaf / n;
    stats.meanMinPrimsInLeaf = stats.totalMinPrimsInLeaf / n;
    stats.meanMaxDepth = stats.totalMaxDepth / n;
    stats.meanSurfaceArea = stats.totalSurfaceArea / n;
    stats.meanNumInterior = stats.totalNumInterior / n;
    stats.meanNumLeaves = stats.totalNumLeaves / n;
    stats.meanCost = stats.totalCost / n;

    // Handle standard deviation statistics
    for(size_t i = 0 ; i < numTrees ; ++i){
        KDTreeNodeRoot *root = (KDTreeNodeRoot *)
            kdgrove->getTreePointer(i)->root.get();
        stats.stdevBuildingTime += std::pow(
            buildingTimes[i]-stats.meanBuildingTime, 2
        );
        stats.stdevTreePrimitives += std::pow(
            treePrimitives[i]-stats.meanTreePrimitives, 2
        );
        stats.stdevMaxPrimsInLeaf = std::pow(
            root->stats_maxNumPrimsInLeaf-stats.meanMaxPrimsInLeaf, 2
        );
        stats.stdevMinPrimsInLeaf = std::pow(
            root->stats_minNumPrimsInLeaf-stats.meanMinPrimsInLeaf, 2
        );
        stats.stdevMaxDepth = std::pow(
            root->stats_maxDepthReached-stats.meanMaxDepth, 2
        );
        stats.stdevSurfaceArea = std::pow(
            root->surfaceArea-stats.meanSurfaceArea, 2
        );
        stats.stdevNumInterior = std::pow(
            root->stats_numInterior-stats.meanNumInterior, 2
        );
        stats.stdevNumLeaves = std::pow(
            root->stats_numLeaves-stats.meanNumLeaves, 2
        );
        stats.stdevCost = std::pow(
            root->stats_totalCost-stats.meanCost, 2
        );
    }
    stats.stdevBuildingTime = std::sqrt(stats.stdevBuildingTime)/n;
    stats.stdevTreePrimitives = std::sqrt(stats.stdevTreePrimitives)/n;
    stats.stdevMaxPrimsInLeaf = std::sqrt(stats.stdevMaxPrimsInLeaf)/n;
    stats.stdevMinPrimsInLeaf = std::sqrt(stats.stdevMinPrimsInLeaf)/n;
    stats.stdevMaxDepth = std::sqrt(stats.stdevMaxDepth)/n;
    stats.stdevSurfaceArea = std::sqrt(stats.stdevSurfaceArea)/n;
    stats.stdevNumInterior = std::sqrt(stats.stdevNumInterior)/n;
    stats.stdevNumLeaves = std::sqrt(stats.stdevNumLeaves)/n;
    stats.stdevCost = std::sqrt(stats.stdevCost)/n;
}
// ***  UTIL BUILDING METHODS  *** //
// ******************************* //
shared_ptr<KDGrove> KDGroveFactory::makeCommon(
    vector<shared_ptr<ScenePart>> parts,
    bool const safe,
    bool const computeKDGroveStats,
    bool const reportKDGroveStats,
    bool const computeKDTreeStats,
    bool const reportKDTreeStats
){
    // Prepare KDGrove building
    shared_ptr<KDGrove> kdgrove = make_shared<KDGrove>();
    kdgrove->setStats(
        computeKDGroveStats ? make_shared<KDGroveStats>() : nullptr
    );
    vector<double> buildingTimes;
    vector<int> numPrimitives;

    // Build each KDTree
    for(shared_ptr<ScenePart> &part : parts){
        TimeWatcher tw;
        tw.start();
        shared_ptr<KDTreeNodeRoot> kdtree = shared_ptr<KDTreeNodeRoot>(
            safe ?
            kdtf->makeFromPrimitives(
                part->mPrimitives, computeKDTreeStats, reportKDTreeStats
            ) :
            kdtf->makeFromPrimitivesUnsafe(
                part->mPrimitives, computeKDTreeStats, reportKDTreeStats
            )
        );
        KDGroveSubject *subject = nullptr;
        if(part->getType()==ScenePart::ObjectType::DYN_MOVING_OBJECT){
            subject = (DynMovingObject *) part.get();
            subject->registerObserverGrove(kdgrove);
        }
        kdgrove->addSubject(
            subject,
            make_shared<GroveKDTreeRaycaster>(kdtree)
        );
        tw.stop();
        buildingTimes.push_back(tw.getElapsedDecimalSeconds());
        numPrimitives.push_back(part->getPrimitives().size());
    }

    // Compute and report KDGrove stats (if requested)
    if(computeKDGroveStats){
        handleKDGroveStats(kdgrove, buildingTimes, numPrimitives);
        if(reportKDGroveStats) logging::INFO(kdgrove->getStats()->toString());
    }

    // Return built kdgrove
    return kdgrove;
}
shared_ptr<KDGrove> KDGroveFactory::makeFull(
    vector<shared_ptr<ScenePart>> parts,
    bool const safe,
    bool const computeKDGroveStats,
    bool const reportKDGroveStats,
    bool const computeKDTreeStats,
    bool const reportKDTreeStats
){
    return makeCommon(
        parts,
        safe,
        computeKDGroveStats,
        reportKDGroveStats,
        computeKDTreeStats,
        reportKDTreeStats
    );
}

shared_ptr<KDGrove> KDGroveFactory::makeMergeNonMoving(
    vector<shared_ptr<ScenePart>> _parts,
    bool const safe,
    bool const computeKDGroveStats,
    bool const reportKDGroveStats,
    bool const computeKDTreeStats,
    bool const reportKDTreeStats
){
    // Prepare merged non moving scene parts
    vector<shared_ptr<ScenePart>> parts;
    vector<Primitive *> mergedPrimitives;
    for(shared_ptr<ScenePart> part : _parts){
        // Consider moving objects directly
        if(part->getType()==ScenePart::ObjectType::DYN_MOVING_OBJECT){
            parts.push_back(part);
            continue;
        }
        // Extract primitives from non moving objects
        vector<Primitive *> const &partPrimitives = part->getPrimitives();
        mergedPrimitives.insert(
            mergedPrimitives.end(),
            partPrimitives.cbegin(),
            partPrimitives.cend()
        );
    }
    // Insert merged scene part
    shared_ptr<ScenePart> mergedPart = make_shared<ScenePart>();
    mergedPart->setPrimitives(mergedPrimitives);
    parts.push_back(mergedPart);

    // Common make
    return makeCommon(
        parts,
        safe,
        computeKDGroveStats,
        reportKDGroveStats,
        computeKDTreeStats,
        reportKDTreeStats
    );
}
