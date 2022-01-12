#pragma once

#include <SimpleKDTreeFactory.h>
#include <SimpleKDTreeGeometricStrategy.h>
#include <SAHKDTreeFactory.h>
#include <SAHKDTreeGeometricStrategy.h>
#include <AxisSAHKDTreeFactory.h>
#include <AxisSAHKDTreeGeometricStrategy.h>
#include <FastSAHKDTreeFactory.h>
#include <FastSAHKDTreeGeometricStrategy.h>
#include <MultiThreadKDTreeFactory.h>
#include <MultiThreadSAHKDTreeFactory.h>

#include <memory>

using std::shared_ptr;
using std::make_shared;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Collection of static methods to simplify making of different
 *  KDTree factories
 */
class KDTreeFactoryMaker {
private:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    KDTreeFactoryMaker() = delete;
    virtual ~KDTreeFactoryMaker() = default;

    // ***  STATIC ATTRIBUTES : DECLARE *** //
    // ************************************ //
    /**
     * @brief Number of threads available in the system
     */
    static size_t const sysThreads;


public:
    // ***  STATIC MAKE METHODS  *** //
    // ***************************** //
    /**
     * @brief Build a simple KDTree factory
     * @return Built simple KDTree factory
     * @see SimpleKDTreeFactory
     */
    static inline shared_ptr<SimpleKDTreeFactory> makeSimple(){
        return make_shared<SimpleKDTreeFactory>();
    }
    /**
     * @brief Build a multi-thread simple KDTree factory
     * @param kdtNodeJobs Number of threads for node-level parallelization
     * @param kdtGeomJobs Number of threads for geometry-level parallelization
     * @return Built multi-thread simple KDTree factory
     * @see KDTreeFactoryMaker::makeSimple
     * @see SimpleKDTreeFactory
     * @see SimpleKDTreeGeometricStrategy
     * @see MultiThreadKDTreeFactory
     */
    static inline shared_ptr<MultiThreadKDTreeFactory> makeSimpleMultiThread(
        size_t const kdtNodeJobs,
        size_t const kdtGeomJobs
    ){
        shared_ptr<SimpleKDTreeFactory> kdtf = makeSimple();
        shared_ptr<SimpleKDTreeGeometricStrategy> gs = \
            make_shared<SimpleKDTreeGeometricStrategy>(*kdtf);
        return make_shared<MultiThreadKDTreeFactory>(
            kdtf, gs, kdtNodeJobs, kdtGeomJobs
        );
    }
    /**
     * @brief Build a multi-thread simple KDTree factory assuming the number
     *  of threads for geometry-level parallelization is the same than the
     *  number of threads for node-level parallelization
     * @see makeSimpleMultiThread(size_t const, size_t const)
     */
    static inline shared_ptr<MultiThreadKDTreeFactory> makeSimpleMultiThread(
        size_t const kdtNodeJobs
    ){
        return makeSimpleMultiThread(kdtNodeJobs, kdtNodeJobs);
    }
    /**
     * @brief Build a multi-thread simple KDTree factory assuming the number of
     *  threads for geometry and node levels parallelization to be the number
     *  of system threads
     * @see makeSimpleMultiThread(size_t const, size_t const)
     * @see KDTreeFactoryMaker::sysThreads
     */
    static inline shared_ptr<MultiThreadKDTreeFactory> makeSimpleMultiThread(){
        return makeSimpleMultiThread(sysThreads, sysThreads);
    }

    /**
     * @brief Build a SAH KDTree factory
     * @param lossNodes How many split nodes check during iterative finding
     *  of best split position
     * @return Built SAH KDTree factory
     * @see SAHKDTreeFactory
     */
    static inline shared_ptr<SAHKDTreeFactory> makeSAH(size_t const lossNodes){
        return make_shared<SAHKDTreeFactory>(lossNodes);
    }
    /**
     * @brief Build a multi-thread SAH KDTree factory
     * @param kdtNodeJobs Number of threads for node-level parallelization
     * @param kdtGeomJobs Number of threads for geometry-level parallelization
     * @return Built multi-thread SAH KDTree factory
     * @see KDTreeFactoryMaker::makeSAH
     * @see SAHKDTreeFactory
     * @see SAHKDTreeGeometricStrategy
     * @see MultiThreadSAHKDTreeFactory
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory> makeSAHMultiThread(
        size_t const lossNodes,
        size_t const kdtNodeJobs,
        size_t const kdtGeomJobs
    ){
        shared_ptr<SAHKDTreeFactory> kdtf = makeSAH(lossNodes);
        shared_ptr<SAHKDTreeGeometricStrategy> gs = \
            make_shared<SAHKDTreeGeometricStrategy>(*kdtf);
        return make_shared<MultiThreadSAHKDTreeFactory>(
            kdtf, gs, kdtNodeJobs, kdtGeomJobs
        );
    }
    /**
     * @brief Build a multi-thread SAH KDTree factory assuming the number of
     *  threads for geometry-level parallelization is the same than the
     *  number of threads for node-level parallelization
     * @see makeSAHMultiThread(size_t const, size_t const, size_t const)
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory> makeSAHMultiThread(
        size_t const lossNodes,
        size_t const kdtNodeJobs
    ){
        return makeSAHMultiThread(lossNodes, kdtNodeJobs, kdtNodeJobs);
    }
    /**
     * @brief Build a multi-thread SAH KDTree factory assuming the number of
     *  threads for geometry and node levels parallelization to be the number
     *  of system threads
     * @see makeSAHMultiThread(size_t const, size_t const, size_t const)
     * @see KDTreeFactoryMaker::sysThreads
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory> makeSAHMultiThread(
        size_t const lossNodes
    ){
        return makeSAHMultiThread(lossNodes, sysThreads, sysThreads);
    }
    /**
     * @brief Build a multi-thread SAH KDTree factory with system threads and
     *  default loss nodes
     * @see makeSAHMultiThread(size_t const, size_t const, size_t const)
     * @see KDTreeFactoryMaker::sysThreads
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory> makeSAHMultiThread(){
        return makeSAHMultiThread(21, sysThreads, sysThreads);
    }

    /**
     * @brief Build an Axis SAH KDTree factory
     * @param lossNodes How many split nodes check during iterative finding
     *  of best split position
     * @return Built Axis SAH KDTree Factory
     * @see AxisSAHKDTreeFactory
     */
    static inline shared_ptr<AxisSAHKDTreeFactory> makeAxisSAH(
        size_t const lossNodes
    ){
        return make_shared<AxisSAHKDTreeFactory>(lossNodes);
    }
    /**
     * @brief Build a multi-thread axis SAH KDTree factory
     * @param kdtNodeJobs Number of threads for node-level parallelization
     * @param kdtGeomJobs Number of threads for geometry-level parallelization
     * @return Built multi-thread axis SAH KDTree factory
     * @see KDTreeFactoryMaker::makeAxisSAH
     * @see AxisSAHKDTreeFactory
     * @see AxisSAHKDTreeGeometricStrategy
     * @see MultiThreadSAHKDTreeFactory
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory>
    makeAxisSAHMultiThread(
        size_t const lossNodes,
        size_t const kdtNodeJobs,
        size_t const kdtGeomJobs
    ){
        shared_ptr<AxisSAHKDTreeFactory> kdtf = makeAxisSAH(lossNodes);
        shared_ptr<AxisSAHKDTreeGeometricStrategy> gs = \
            make_shared<AxisSAHKDTreeGeometricStrategy>(*kdtf);
        return make_shared<MultiThreadSAHKDTreeFactory>(
            kdtf, gs, kdtNodeJobs, kdtGeomJobs
        );
    }
    /**
     * @brief Build a multi-thread axis SAH KDTree factory assuming the number
     *  of threads for geometry-level parallelization is the same than the
     *  number of threads for node-level parallelization
     * @see makeAxisSAHMultiThread(size_t const, size_t const, size_t const)
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory>
    makeAxisSAHMultiThread(
        size_t const lossNodes,
        size_t const kdtNodeJobs
    ){
        return makeAxisSAHMultiThread(lossNodes, kdtNodeJobs, kdtNodeJobs);
    }
    /**
     * @brief Build a multi-thread axis SAH KDTree factory assuming the number
     *  of threads for geometry and node levels parallelization to be the
     *  number of system threads
     * @see makeAxisSAHMultiThread(size_t const, size_t const, size_t const)
     * @see KDTreeFactoryMaker::sysThreads
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory>
    makeAxisSAHMultiThread(size_t const lossNodes){
        return makeAxisSAHMultiThread(lossNodes, sysThreads, sysThreads);
    }
    /**
     * @brief Build a multi-thread axis SAH KDTree factory with system threads
     *  and default loss nodes
     * @see makeAxisSAHMultiThread(size_t const, size_t const, size_t const)
     * @see KDTreeFactoryMaker::sysThreads
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory>
    makeAxisSAHMultiThread(){
        return makeAxisSAHMultiThread(21, sysThreads, sysThreads);
    }

    /**
     * @brief Build a fast SAH KDTree factory
     * @param lossNodes How many bins must be used during primitives recount
     *  to approximate best split position
     * @return Built fast SAH KDTree factory
     * @see FastSAHKDTreeFactory
     */
    static inline shared_ptr<FastSAHKDTreeFactory> makeFastSAH(
        size_t const lossNodes
    ){
        return make_shared<FastSAHKDTreeFactory>(lossNodes);
    }
    /**
     * @brief Build a multi-thread fast SAH KDTree factory
     * @param kdtNodeJobs Number of threads for node-level parallelization
     * @param kdtGeomJobs Number of threads for geometry-level parallelization
     * @return Built multi-thread fast SAH KDTree factory
     * @see KDTreeFactoryMaker::makeFastSAH
     * @see FastSAHKDTreeFactory
     * @see FastSAHKDTreeGeometricStrategy
     * @see MultiThreadSAHKDTreeFactory
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory>
    makeFastSAHMultiThread(
        size_t const lossNodes,
        size_t const kdtNodeJobs,
        size_t const kdtGeomJobs
    ){
        shared_ptr<FastSAHKDTreeFactory> kdtf = makeFastSAH(lossNodes);
        shared_ptr<FastSAHKDTreeGeometricStrategy> gs = \
            make_shared<FastSAHKDTreeGeometricStrategy>(*kdtf);
        return make_shared<MultiThreadSAHKDTreeFactory>(
            kdtf, gs, kdtNodeJobs, kdtGeomJobs
        );
    }
    /**
     * @brief Build a multi-thread fast SAH KDTree factory assuming the number
     *  of threads for geometry-level parallelization is the same than the
     *  number of threads for node-level parallelization
     * @see makeFastSAHMultiThread(size_t const, size_t const, size_t const)
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory>
    makeFastSAHMultiThread(
        size_t const lossNodes,
        size_t const kdtNodeJobs
    ){
        return makeFastSAHMultiThread(lossNodes, kdtNodeJobs, kdtNodeJobs);
    }
    /**
     * @brief Build a multi-thread fast SAH KDTree factory assuming the number
     *  of threads for geometry and node levels parallelization to be the
     *  number of system threads
     * @see makeFastSAHMultiThread(size_t const, size_t const, size_t const)
     * @see KDTreeFactoryMaker::sysThreads
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory>
    makeFastSAHMultiThread(size_t const lossNodes){
        return makeFastSAHMultiThread(lossNodes, sysThreads, sysThreads);
    }
    /**
     * @brief Build a multi-thread fast SAH KDTree factory with system threads
     *  and default loss nodes
     * @see makeFastSAHMultiThread(size_t const, size_t const, size_t const)
     * @see KDTreeFactoryMaker::sysThreads
     */
    static inline shared_ptr<MultiThreadSAHKDTreeFactory>
    makeFastSAHMultiThread(){
        return makeFastSAHMultiThread(32, sysThreads, sysThreads);
    }
};

// ***  STATIC ATTRIBUTES : INITIALIZE *** //
// *************************************** //
size_t const KDTreeFactoryMaker::sysThreads = \
    std::thread::hardware_concurrency();

