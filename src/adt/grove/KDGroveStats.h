#pragma once

#include <string>
#include <sstream>
#include <ostream>
#include <vector>

using std::string;
using std::stringstream;
using std::ostream;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Data structure class containing statistics for KDGrove
 */
class KDGroveStats{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The total number of trees composing the KDGrove
     * @see KDGroveStats::numStaticTrees
     * @see KDGroveStats::numDynTrees
     */
    int numTrees;
    /**
     * @brief The number of static trees composing the KDGrove
     * @see KDGroveStats::numTrees
     * @see KDGroveStats::numDynTrees
     */
    int numStaticTrees;
    /**
     * @brief The number of dynamic trees composing the KDGrove
     * @see KDGroveStats::numTrees
     * @see KDGroveStats::numStaticTrees
     */
    int numDynTrees;
    /**
     * @brief Total building time among all built KDTrees (seconds)
     */
    double totalBuildingTime;
    /**
     * @brief Minimum building time among all built KDTrees (seconds)
     */
    double minBuildingTime;
    /**
     * @brief Maximum building time among all built KDTrees (seconds)
     */
    double maxBuildingTime;
    /**
     * @brief Mean building time among all built KDTrees (seconds)
     */
    double meanBuildingTime;
    /**
     * @brief Standard deviation of building time among all built KDTrees
     *  (seconds)
     */
    double stdevBuildingTime;
    /**
     * @brief The total maximum number of primitives among all
     *  leaves for each KDTree (summation)
     * @see KDTreeNodeRoot::stats_maxNumPrimsInLeaf
     * @see KDGroveStats::meanMaxPrimsInLeaf
     * @see KDGroveStats::stdevMaxPrimsInLeaf
     * @see KDGroveStats::minMaxPrimsInLeaf
     * @see KDGroveStats::maxMaxPrimsInLeaf
     */
    int totalMaxPrimsInLeaf;
    /**
     * @brief The minimum maximum number of primitives among all
     *  leaves for each KDTree
     * @see KDTreeNodeRoot::stats_maxNumPrimsInLeaf
     * @see KDGroveStats::totalMaxPrimsInLeaf
     * @see KDGroveStats::meanMaxPrimsInLeaf
     * @see KDGroveStats::stdevMaxPrimsInLeaf
     * @see KDGroveStats::maxMaxPrimsInLeaf
     */
    int minMaxPrimsInLeaf;
    /**
     * @brief The maximum maximum number of primitives among all
     *  leaves for each KDTree
     * @see KDTreeNodeRoot::stats_maxNumPrimsInLeaf
     * @see KDGroveStats::totalMaxPrimsInLeaf
     * @see KDGroveStats::meanMaxPrimsInLeaf
     * @see KDGroveStats::stdevMaxPrimsInLeaf
     * @see KDGroveStats::minMaxPrimsInLeaf
     */
    int maxMaxPrimsInLeaf;
    /**
     * @brief The mean maximum number of primitives among all
     *  leaves for each KDTree
     * @see KDTreeNodeRoot::stats_maxNumPrimsInLeaf
     * @see KDGroveStats::totalMaxPrimsInLeaf
     * @see KDGroveStats::stdevMaxPrimsInLeaf
     * @see KDGroveStats::minMaxPrimsInLeaf
     * @see KDGroveStats::maxMaxPrimsInLeaf
     */
    double meanMaxPrimsInLeaf;
    /**
     * @brief The standard deviation of maximum number of primitives among all
     *  leaves for each KDTree
     * @see KDTreeNodeRoot::stats_maxNumPrimsInLeaf
     * @see KDGroveStats::totalMaxPrimsInLeaf
     * @see KDGroveStats::meanMaxPrimsInLeaf
     * @see KDGroveStats::minMaxPrimsInLeaf
     * @see KDGroveStats::maxMaxPrimsInLeaf
     */
    double stdevMaxPrimsInLeaf;
    /**
     * @brief The total minimum number of primitives among all
     *  leaves for each KDTree (summation)
     * @see KDTreeNodeRoot::stats_minNumPrimsInLeaf
     * @see KDGroveStats::minMinPrimsInLeaf
     * @see KDGroveStats::maxMinPrimsInLeaf
     * @see KDGroveStats::meanMinPrimsInLeaf
     * @see KDGroveStats::stdevMinPrimsInLeaf
     */
    int totalMinPrimsInLeaf;
    /**
     * @brief The minimun minimum number of primitives among all
     *  leaves for each KDTree
     * @see KDTreeNodeRoot::stats_minNumPrimsInLeaf
     * @see KDGroveStats::totalMinPrimsInLeaf
     * @see KDGroveStats::maxMinPrimsInLeaf
     * @see KDGroveStats::meanMinPrimsInLeaf
     * @see KDGroveStats::stdevMinPrimsInLeaf
     */
    int minMinPrimsInLeaf;
    /**
     * @brief The maximum minimum number of primitives among all
     *  leaves for each KDTree
     * @see KDTreeNodeRoot::stats_minNumPrimsInLeaf
     * @see KDGroveStats::minMinPrimsInLeaf
     * @see KDGroveStats::totalMinPrimsInLeaf
     * @see KDGroveStats::meanMinPrimsInLeaf
     * @see KDGroveStats::stdevMinPrimsInLeaf
     */
    int maxMinPrimsInLeaf;
    /**
     * @brief The mean minimum number of primitives among all
     *  leaves for each KDTree
     * @see KDTreeNodeRoot::stats_minNumPrimsInLeaf
     * @see KDGroveStats::minMinPrimsInLeaf
     * @see KDGroveStats::maxMinPrimsInLeaf
     * @see KDGroveStats::totalMinPrimsInLeaf
     * @see KDGroveStats::stdevMinPrimsInLeaf
     */
    double meanMinPrimsInLeaf;
    /**
     * @brief The standard deviation of minimum number of primitives among all
     *  leaves for each KDTree
     * @see KDTreeNodeRoot::stats_minNumPrimsInLeaf
     * @see KDGroveStats::minMinPrimsInLeaf
     * @see KDGroveStats::maxMinPrimsInLeaf
     * @see KDGroveStats::totalMinPrimsInLeaf
     * @see KDGroveStats::meanMinPrimsInLeaf
     */
    double stdevMinPrimsInLeaf;
    /**
     * @brief The total maximum depth among all KDTrees (summation)
     * @see KDTreeNodeRoot::stats_maxDepthReached
     * @see KDGroveStats::minMaxDepth
     * @see KDGroveStats::maxMaxDepth
     * @see KDGroveStats::meanMaxDepth
     * @see KDGroveStats::stdevMaxDepth
     */
    int totalMaxDepth;
    /**
     * @brief The minimum maximum depth among all KDTrees
     * @see KDTreeNodeRoot::stats_maxDepthReached
     * @see KDGroveStats::totalMaxDepth
     * @see KDGroveStats::maxMaxDepth
     * @see KDGroveStats::meanMaxDepth
     * @see KDGroveStats::stdevMaxDepth
     */
    int minMaxDepth;
    /**
     * @brief The maximum maximum depth among all KDTrees
     * @see KDTreeNodeRoot::stats_maxDepthReached
     * @see KDGroveStats::totalMaxDepth
     * @see KDGroveStats::minMaxDepth
     * @see KDGroveStats::meanMaxDepth
     * @see KDGroveStats::stdevMaxDepth
     */
    int maxMaxDepth;
    /**
     * @brief The mean max depth among all KDTrees
     * @see KDTreeNodeRoot::stats_maxDepthReached
     * @see KDGroveStats::totalMaxDepth
     * @see KDGroveStats::minMaxDepth
     * @see KDGroveStats::maxMaxDepth
     * @see KDGroveStats::stdevMaxDepth
     */
    double meanMaxDepth;
    /**
     * @brief The standard deviation of max depth among all KDTrees
     * @see KDTreeNodeRoot::stats_maxDepthReached
     * @see KDGroveStats::totalMaxDepth
     * @see KDGroveStats::minMaxDepth
     * @see KDGroveStats::maxMaxDepth
     * @see KDGroveStats::meanMaxDepth
     */
    double stdevMaxDepth;
    /**
     * @brief The total number of interior nodes among all KDTrees (summation)
     * @see KDTreeNodeRoot::stats_numInterior
     * @see KDGroveStats::minNumInterior
     * @see KDGroveStats::maxNumInterior
     * @see KDGroveStats::meanNumInterior
     * @see KDGroveStats::stdevNumInterior
     */
    int totalNumInterior;
    /**
     * @brief The minimum number of interior nodes among all KDTrees
     * @see KDTreeNodeRoot::stats_numInterior
     * @see KDGroveStats::totalNumInterior
     * @see KDGroveStats::maxNumInterior
     * @see KDGroveStats::meanNumInterior
     * @see KDGroveStats::stdevNumInterior
     */
    int minNumInterior;
    /**
     * @brief The maximum number of interior nodes among all KDTrees
     * @see KDTreeNodeRoot::stats_numInterior
     * @see KDGroveStats::totalNumInterior
     * @see KDGroveStats::minNumInterior
     * @see KDGroveStats::meanNumInterior
     * @see KDGroveStats::stdevNumInterior
     */
    int maxNumInterior;
    /**
     * @brief The mean number of interior nodes among all KDTrees
     * @see KDTreeNodeRoot::stats_numInterior
     * @see KDGroveStats::totalNumInterior
     * @see KDGroveStats::minNumInterior
     * @see KDGroveStats::maxNumInterior
     * @see KDGroveStats::stdevNumInterior
     */
    double meanNumInterior;
    /**
     * @brief The standard deviation of interior nodes count among all KDTrees
     * @see KDTreeNodeRoot::stats_numInterior
     * @see KDGroveStats::totalNumInterior
     * @see KDGroveStats::minNumInterior
     * @see KDGroveStats::maxNumInterior
     * @see KDGroveStats::meanNumInterior
     */
    double stdevNumInterior;
    /**
     * @brief The total number of leaf nodes among all KDTrees (summation)
     * @see KDTreeNodeRoot::stats_numLeaves
     * @see KDGroveStats::minNumLeaves
     * @see KDGroveStats::maxNumLeaves
     * @see KDGroveStats::meanNumLeaves
     * @see KDGroveStats::stdevNumLeaves
     */
    int totalNumLeaves;
    /**
     * @brief The minimum number of leaf nodes among all KDTrees
     * @see KDTreeNodeRoot::stats_numLeaves
     * @see KDGroveStats::totalNumLeaves
     * @see KDGroveStats::maxNumLeaves
     * @see KDGroveStats::meanNumLeaves
     * @see KDGroveStats::stdevNumLeaves
     */
    int minNumLeaves;
    /**
     * @brief The maximum number of leaf nodes among all KDTrees
     * @see KDTreeNodeRoot::stats_numLeaves
     * @see KDGroveStats::totalNumLeaves
     * @see KDGroveStats::minNumLeaves
     * @see KDGroveStats::meanNumLeaves
     * @see KDGroveStats::stdevNumLeaves
     */
    int maxNumLeaves;
    /**
     * @brief The mean number of leaf nodes among all KDTrees
     * @see KDTreeNodeRoot::stats_numLeaves
     * @see KDGroveStats::totalNumLeaves
     * @see KDGroveStats::minNumLeaves
     * @see KDGroveStats::maxNumLeaves
     * @see KDGroveStats::stdevNumLeaves
     */
    double meanNumLeaves;
    /**
     * @brief The standard deviation of leaf nodes count among all KDTrees
     * @see KDTreeNodeRoot::stats_numLeaves
     * @see KDGroveStats::totalNumLeaves
     * @see KDGroveStats::minNumLeaves
     * @see KDGroveStats::maxNumLeaves
     * @see KDGroveStats::meanNumLeaves
     */
    double stdevNumLeaves;
    /**
     * @brief Total cost among all KDTrees (summation)
     * @see KDTreeNodeRoot::stats_totalCost
     * @see KDGroveStats::minCost
     * @see KDGroveStats::maxCost
     * @see KDGroveStats::meanCost
     * @see KDGroveStats::stdevCost
     */
    double totalCost;
    /**
     * @brief The minimum tree cost among all KDTrees
     * @see KDTreeNodeRoot::stats_totalCost
     * @see KDGroveStats::totalCost
     * @see KDGroveStats::maxCost
     * @see KDGroveStats::meanCost
     * @see KDGroveStats::stdevCost
     */
    double minCost;
    /**
     * @biref The maximum tree cost among all KDTrees
     * @see KDTreeNodeRoot::stats_totalCost
     * @see KDGroveStats::totalCost
     * @see KDGroveStats::minCost
     * @see KDGroveStats::meanCost
     * @see KDGroveStats::stdevCost
     */
    double maxCost;
    /**
     * @brief The mean cost among all KDTrees
     * @see KDTreeNodeRoot::stats_totalCost
     * @see KDGroveStats::totalCost
     * @see KDGroveStats::minCost
     * @see KDGroveStats::maxCost
     * @see KDGroveStats::stdevCost
     */
    double meanCost;
    /**
     * @brief The standard deviation of cost among all KDTrees
     * @see KDTreeNodeRoot::stats_totalCost
     * @see KDGroveStats::totalCost
     * @see KDGroveStats::minCost
     * @see KDGroveStats::maxCost
     * @see KDGroveStats::meanCost
     */
    double stdevCost;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for KDGrove stats
     */
    KDGroveStats() :
        numTrees(0),
        numStaticTrees(0),
        numDynTrees(0),
        totalBuildingTime(0),
        minBuildingTime(0),
        maxBuildingTime(0),
        meanBuildingTime(0),
        stdevBuildingTime(0),
        totalMaxPrimsInLeaf(0),
        minMaxPrimsInLeaf(0),
        maxMaxPrimsInLeaf(0),
        meanMaxPrimsInLeaf(0),
        stdevMaxPrimsInLeaf(0),
        totalMinPrimsInLeaf(0),
        minMinPrimsInLeaf(0),
        maxMinPrimsInLeaf(0),
        meanMinPrimsInLeaf(0),
        stdevMinPrimsInLeaf(0),
        totalMaxDepth(0),
        minMaxDepth(0),
        maxMaxDepth(0),
        meanMaxDepth(0),
        stdevMaxDepth(0),
        totalNumInterior(0),
        minNumInterior(0),
        maxNumInterior(0),
        meanNumInterior(0),
        stdevNumInterior(0),
        totalNumLeaves(0),
        minNumLeaves(0),
        maxNumLeaves(0),
        meanNumLeaves(0),
        stdevNumLeaves(0),
        totalCost(0),
        minCost(0),
        maxCost(0),
        meanCost(0),
        stdevCost(0)
    {}
    virtual ~KDGroveStats() = default;

    // ***  TO STRING  *** //
    // ******************* //
    /**
     * @brief Build a string representation of the KDGrove stats
     * @return String representation of KDGrove stats
     */
    string toString() const {
        stringstream ss;
        ss  << "KDGrove stats:\n"
            << "\tNumber of trees: " << numTrees << "\n"
            << "\tNumber of static trees: " << numStaticTrees << "\n"
            << "\tNumber of dynamic trees: " << numDynTrees << "\n"
            << "\tStatistics (min, max, total, mean, stdev):\n"
            << "\t\tBuilding time: ("
                << minBuildingTime << ", "
                << maxBuildingTime << ", "
                << totalBuildingTime << ", "
                << meanBuildingTime << ", "
                << stdevBuildingTime << ")\n"
            << "\t\tMax primitives in leaf: ("
                << minMaxPrimsInLeaf << ", "
                << maxMaxPrimsInLeaf << ", "
                << totalMaxPrimsInLeaf << ", "
                << meanMaxPrimsInLeaf << ", "
                << stdevMaxPrimsInLeaf << ")\n"
            << "\t\tMin primitives in leaf: ("
                << minMinPrimsInLeaf << ", "
                << maxMinPrimsInLeaf << ", "
                << totalMinPrimsInLeaf << ", "
                << meanMinPrimsInLeaf << ", "
                << stdevMinPrimsInLeaf << ")\n"
            << "\t\tMaximum depth: ("
                << minMaxDepth << ", "
                << maxMaxDepth << ", "
                << totalMaxDepth << ", "
                << meanMaxDepth << ", "
                << stdevMaxDepth << ")\n"
            << "\t\tNumber of interior nodes: ("
                << minNumInterior << ", "
                << maxNumInterior << ", "
                << totalNumInterior << ", "
                << meanNumInterior << ", "
                << stdevNumInterior << ")\n"
            << "\t\tNumber of leaf nodes: ("
                << minNumLeaves << ", "
                << maxNumLeaves << ", "
                << totalNumLeaves << ", "
                << meanNumLeaves << ", "
                << stdevNumLeaves << ")\n"
            << "\t\tTree cost: ("
                << minCost << ", "
                << maxCost << ", "
                << totalCost << ", "
                << meanCost << ", "
                << stdevCost << ")\n"
        ;
        return ss.str();
    }

    /**
     * @brief Support for operator<< on output streams
     */
    friend ostream& operator<<(ostream &out, KDGroveStats const &stats){
        out << stats.toString();
        return out;
    }
};