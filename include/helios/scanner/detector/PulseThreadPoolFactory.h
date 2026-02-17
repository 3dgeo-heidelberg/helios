#pragma once

#include <helios/scanner/detector/PulseThreadPool.h>
#include <helios/scanner/detector/PulseThreadPoolInterface.h>
#include <helios/scanner/detector/PulseWarehouseThreadPool.h>
#include <helios/util/HeliosException.h>

#include <memory>
#include <sstream>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Factory to build different types of pulse thread pools
 * @see PulseThreadPool
 * @see PulseWarehouseThreadPool
 * @see PulseThreadPoolInterface
 */
class PulseThreadPoolFactory
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The parallelization strategy defining the thread pool
   *
   * Strategy 0 is a chunk based strategy while strategy 1 is a warehouse
   *  based strategy
   */
  int parallelizationStrategy;
  /**
   * @brief How many threads the thread pool should use
   * @see ThreadPool::pool_size
   */
  std::size_t poolSize;
  /**
   * @brief The accuracy of the detector in meters
   * @see AbstractDetector::cfg_device_accuracy_m
   */
  double deviceAccuracy;
  /**
   * @brief The size of chunks handled by the thread pool, if needed
   * @see TaskDropper::maxTasks
   */
  int chunkSize;
  /**
   * @brief The warehouse factor for the thread pool, if needed.
   * The maximum number of tasks for a TaskWarehouse will be the pool size
   *  multiplied by this factor
   * @see TaskWarehouse::maxTasks
   */
  int warehouseFactor;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor
   */
  PulseThreadPoolFactory(int const parallelizationStrategy,
                         std::size_t const poolSize,
                         double const deviceAccuracy,
                         int const chunkSize,
                         int const warehouseFactor = 1)
    : parallelizationStrategy(parallelizationStrategy)
    , poolSize(poolSize)
    , deviceAccuracy(deviceAccuracy)
    , chunkSize(chunkSize)
    , warehouseFactor(warehouseFactor)
  {
  }
  virtual ~PulseThreadPoolFactory() = default;

  // ***  FACTORY METHODS  *** //
  // ************************* //
  /**
   * @brief Build the pulse thread pool corresponding with current factory
   *  configuration/state
   * @return Built pulse thread pool
   */
  std::shared_ptr<PulseThreadPoolInterface> makePulseThreadPool()
  {
    if (parallelizationStrategy == 0) {
      return makeBasicPulseThreadPool();
    } else if (parallelizationStrategy == 1) {
      return makePulseWarehouseThreadPool();
    } else {
      std::stringstream ss;
      ss << "Unexpected parallelization strategy: " << parallelizationStrategy;
      throw HeliosException(ss.str());
    }
  }

  /**
   * @brief Build a basic pulse thread pool
   * @return Built basic pulse thread pool
   * @see PulseThreadPool
   */
  inline std::shared_ptr<PulseThreadPool> makeBasicPulseThreadPool() const
  {
    return std::make_shared<PulseThreadPool>(
      poolSize, deviceAccuracy, chunkSize < 0);
  }

  /**
   * @brief Build a warehouse based pulse thread pool
   * @return Built warehouse based pulse thread pool
   * @see PulseWarehouseThreadPool
   */
  inline std::shared_ptr<PulseWarehouseThreadPool>
  makePulseWarehouseThreadPool() const
  {
    return std::make_shared<PulseWarehouseThreadPool>(
      poolSize, deviceAccuracy, poolSize * warehouseFactor);
  }
};
