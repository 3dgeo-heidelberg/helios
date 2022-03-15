#pragma once

// Include boost ASIO preventing windows conflicts ---
#if defined(_WIN32) || defined(_WIN64)  // If using windows
/*#define WIN32_LEAN_AND_MEAN
#include <boost/asio.hpp>
#include <SDKDDKVer.h>
#include <windows.h>*/

#else // If not using windows
#include <boost/asio.hpp>

#endif
// --- Include boost ASIO preventing windows conflicts
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <logging.hpp>
#include <sstream>

/**
 * @version 1.0
 * @brief Base class providing core implementation of a thread pool to deal
 *  with multi threading tasks
 */
class ThreadPool{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Instance of boost input/output service for asynchronous data
     *  processing
     */
    boost::asio::io_service io_service_;
	/**
	 * @brief Instance of work guard to report the io service when it has
	 *  pending tasks
	 * @see ThreadPool::io_service_
	 */
    boost::asio::executor_work_guard<boost::asio::io_service::executor_type>
	    work_;
	/**
	 * @brief Size of thread pool (number of threads)
	 */
    std::size_t pool_size;
    /**
     * @brief Group of threads
     */
	boost::thread_group threads_;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Thread pool constructor
     * @see ThreadPool::pool_size
     */
    explicit ThreadPool(std::size_t const _pool_size) :
        work_(boost::asio::make_work_guard(io_service_)),
        pool_size(_pool_size)
    {
        // Create threads
        for (std::size_t i = 0; i < pool_size; ++i){
            threads_.create_thread(
                [&] () -> boost::asio::io_context::count_type{
                    return io_service_.run();
                }
            );
        }
    }

    virtual ~ThreadPool(){
        // Force all threads to return from io_service::run().
        io_service_.stop();

        // Suppress all exceptions.
        try{
            threads_.join_all();
        }
        catch (const std::exception&) {}
    }

public:
    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Obtain the thread pool size
     * @return Thread pool size
     * @see ThreadPool::pool_size
     */
    virtual inline std::size_t getPoolSize() const {return pool_size;}


};
