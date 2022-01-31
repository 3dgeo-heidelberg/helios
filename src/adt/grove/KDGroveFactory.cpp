#include <KDGroveFactory.h>
#include <KDGroveStats.h>
#include <TimeWatcher.h>


using std::shared_ptr;
using std::make_shared;

// ***  K-DIMENSIONAL GROVE FACTORY METHODS  *** //
// ********************************************* //
shared_ptr<KDGrove> KDGroveFactory::makeFromSceneParts(
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
    vector<double> buildingTimes; // TODO Rethink : Generate time measures

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
        BasicDynGroveSubject *subject = nullptr;
        if(part->getType()==ScenePart::ObjectType::DYN_MOVING_OBJECT){
            subject = (DynMovingObject *) part.get();
        }
        kdgrove->addSubject(
            subject,
            make_shared<GroveKDTreeRaycaster>(kdtree)
        );
        tw.stop();
        buildingTimes.push_back(tw.getElapsedDecimalSeconds());
    }

    // Compute and report KDGrove stats (if requested)
    if(computeKDGroveStats){
        handleKDGroveStats(kdgrove, buildingTimes);
        if(reportKDGroveStats) logging::INFO(kdgrove->getStats()->toString());
    }

    // Return built kdgrove
    return kdgrove;
}
// ***  STATISTICS METHODS  *** //
// **************************** //
void KDGroveFactory::handleKDGroveStats(
    shared_ptr<KDGrove> kdgrove,
    vector<double> &buildingTimes
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
            stats.minMaxPrimsInLeaf = root->stats_maxNumPrimsInLeaf;
            stats.maxMaxPrimsInLeaf = root->stats_maxNumPrimsInLeaf;
            stats.minMinPrimsInLeaf = root->stats_minNumPrimsInLeaf;
            stats.maxMinPrimsInLeaf = root->stats_minNumPrimsInLeaf;
            stats.minMaxDepth = root->stats_maxDepthReached;
            stats.maxMaxDepth = root->stats_maxDepthReached;
            stats.minNumInterior = root->stats_numInterior;
            stats.maxNumInterior = root->stats_numInterior;
            stats.minNumLeaves = root->stats_numLeaves;
            stats.maxNumLeaves = root->stats_numLeaves;
            stats.minCost = root->stats_totalCost;
            stats.maxCost = root->stats_totalCost;
        }
        else{ // Update value for min/max
            stats.minBuildingTime = std::min<int>(
                stats.minBuildingTime, buildingTimes[i]
            );
            stats.maxBuildingTime = std::max<int>(
                stats.maxBuildingTime, buildingTimes[i]
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
            stats.minCost = std::min<int>(
                stats.minCost, root->stats_totalCost
            );
            stats.maxCost = std::max<int>(
                stats.maxCost, root->stats_totalCost
            );
        }
        // Handle total statistics
        stats.totalBuildingTime += buildingTimes[i];
        stats.totalMaxPrimsInLeaf += root->stats_maxNumPrimsInLeaf;
        stats.totalMinPrimsInLeaf += root->stats_minNumPrimsInLeaf;
        stats.totalMaxDepth += root->stats_maxDepthReached;
        stats.totalNumInterior += root->stats_numInterior;
        stats.totalNumLeaves += root->stats_numLeaves;
        stats.totalCost += root->stats_totalCost;
    }
    // Handle mean statistics
    double const n = (double) stats.numTrees;
    stats.meanBuildingTime = stats.totalBuildingTime / n;
    stats.meanMaxPrimsInLeaf = stats.totalMaxPrimsInLeaf / n;
    stats.meanMinPrimsInLeaf = stats.totalMinPrimsInLeaf / n;
    stats.meanMaxDepth = stats.totalMaxDepth / n;
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
        stats.stdevMaxPrimsInLeaf = std::pow(
            root->stats_maxNumPrimsInLeaf-stats.meanMaxPrimsInLeaf, 2
        );
        stats.stdevMinPrimsInLeaf = std::pow(
            root->stats_minNumPrimsInLeaf-stats.meanMinPrimsInLeaf, 2
        );
        stats.stdevMaxDepth = std::pow(
            root->stats_maxDepthReached-stats.meanMaxDepth, 2
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
    stats.stdevMaxPrimsInLeaf = std::sqrt(stats.stdevMaxPrimsInLeaf)/n;
    stats.stdevMinPrimsInLeaf = std::sqrt(stats.stdevMinPrimsInLeaf)/n;
    stats.stdevMaxDepth = std::sqrt(stats.stdevMaxDepth)/n;
    stats.stdevNumInterior = std::sqrt(stats.stdevNumInterior)/n;
    stats.stdevNumLeaves = std::sqrt(stats.stdevNumLeaves)/n;
    stats.stdevCost = std::sqrt(stats.stdevCost)/n;
}
