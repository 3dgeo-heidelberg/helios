#pragma once

#include <KDTreeFactory.h>
#include <SimpleKDTreeFactory.h>
#include <SAHKDTreeFactory.h>
#include <AxisSAHKDTreeFactory.h>
#include <FastSAHKDTreeFactory.h>
#include <MultiThreadKDTreeFactory.h>
#include <MultiThreadSAHKDTreeFactory.h>
#include <KDGrove.h>

#include <memory>
#include <vector>

using std::shared_ptr;
using std::vector;

class KDGroveFactory {
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a KDGrove factory to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for hte K dimensional grove factory
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version){
        // Register KDTree factories
        ar.template register_type<SimpleKDTreeFactory>();
        ar.template register_type<SAHKDTreeFactory>();
        ar.template register_type<AxisSAHKDTreeFactory>();
        ar.template register_type<FastSAHKDTreeFactory>();
        ar.template register_type<MultiThreadKDTreeFactory>();
        ar.template register_type<MultiThreadSAHKDTreeFactory>();

        // Serialization itself
        // TODO Rethink serialization
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The KDTree factory used to build trees composing the grove
     */
    shared_ptr<KDTreeFactory> kdtf;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief K dimensional grove factory default constructor
     */
    KDGroveFactory(shared_ptr<KDTreeFactory> kdtf) : kdtf(kdtf) {}
    virtual ~KDGroveFactory() = default;

    // ***  K-DIMENSIONAL GROVE FACTORY METHODS  *** //
    // ********************************************* //
    /**
     * @brief Bulid a KDGrove from given scene parts
     * @param parts Scene parts to build KDGrove from
     * @param computeKDGroveStats If true, KDGrove stats will be computed. If
     *  false, they will not
     * @param reportKDGroveStats If true, KDGrove stats will be reported. If
     *  false, they will not
     * @param computeKDTreeStats  If true, stats for each KDTree will be
     *  computed. If false, they will not
     * @param reportKDTreeStats If true, stats for each KDTree will be
     *  reported. If false, they will not
     * @return Build KDGrove
     * @see KDGrove
     */
    virtual shared_ptr<KDGrove> makeFromSceneParts(
        vector<shared_ptr<ScenePart>> parts,
        bool const safe=false,
        bool const computeKDGroveStats=false,
        bool const reportKDGroveStats=false,
        bool const computeKDTreeStats=false,
        bool const reportKDTreeStats=false
    );

    // ***  STATISTICS METHODS  *** //
    // **************************** //
    /**
     * @brief Method to handle the update of KDGrove stats after all KDTrees
     *  and the KDGrove itself have been created
     * @param kdgrove The KDGrove which stats must be computed
     * @param buildingTimes The building time for each tree inside the KDGrove
     */
    void handleKDGroveStats(
        shared_ptr<KDGrove> kdgrove,
        vector<double> &buildingTimes
    );
};