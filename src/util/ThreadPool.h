#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <noise/RandomnessGenerator.h>
#include <UniformNoiseSource.h>
#include <logging.hpp>
#include <sstream>

/**
 * @brief Class representing a thread pool to deal with multi threading tasks
 */
class thread_pool{
private:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Instance of boost input/output service for asynchronous data
     *  processing
     */
	boost::asio::io_service io_service_;
	/**
	 * @brief Instance of work to report the io service when it has pending
	 *  tasks
	 * @see thread_pool::io_service_
	 */
	boost::asio::io_service::work work_;
	/**
	 * @brief Size of thread pool (number of threads)
	 */
    std::size_t pool_size;
    /**
     * @brief Group of threads
     */
	boost::thread_group threads_;
	/**
	 * @brief Number of available threads, those which are not currently
	 *  performing a task
	 */
	std::size_t available_;
	/**
	 * @brief Mutex to handle concurrent tasks
	 */
	boost::mutex mutex_;
    /**
     * @brief Condition var to handle tasks dispatching depending on
     *  available threads
     */
	boost::condition_variable cond_;

	/**
	 * Alpha prime matrices for MarquardtFitter.
	 * One per possible thread, to avoid spamming reallocations
	 */
	std::vector<std::vector<double>> *apMatrices;

    /**
     * @brief First randomness generators (general purpose), one per thread
     */
	RandomnessGenerator<double> *randGens; // General purpose
	/**
	 * @brief Second randomness generators (to substitute old box muller), one
	 *  per thread
	 */
	RandomnessGenerator<double> *randGens2; // To substitute BoxMuller
	/**
	 * @brief Intersection handling noise sources, one per thread
	 */
    UniformNoiseSource<double> *intersectionHandlingNoiseSources;

    /**
     * @brief Array of flags specifying availability of resource sets
     *
     * The ith element of this array correspond to the ith resource for any
     *  resource set/array
     */
    bool *resourceSetAvailable;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Thread pool constructor
     * @see thread_pool::pool_size
     * @param deviceAccuracy Parameter used to handle randomness generation
     *  impact on simulation results
     */
    explicit thread_pool(std::size_t _pool_size, double deviceAccuracy)
        : work_(io_service_),
          pool_size(_pool_size),
          available_(_pool_size)
    {
        apMatrices = new std::vector<std::vector<double>>[pool_size];
        randGens = new RandomnessGenerator<double>[pool_size];
        randGens2 = new RandomnessGenerator<double>[pool_size];
        intersectionHandlingNoiseSources =
            new UniformNoiseSource<double>[pool_size];
        resourceSetAvailable = new bool[pool_size];


        for (std::size_t i = 0; i < pool_size; ++i)
        {
            resourceSetAvailable[i] = true;
            randGens[i] = *DEFAULT_RG;
            randGens[i].computeUniformRealDistribution(0.0, 1.0);
            randGens[i].computeNormalDistribution(0.0, deviceAccuracy);
            randGens2[i] = *DEFAULT_RG;
            randGens2[i].computeNormalDistribution(0.0, 1.0);
            intersectionHandlingNoiseSources[i] = UniformNoiseSource<double>(
                *DEFAULT_RG, 0.0, 1.0
            );
            threads_.create_thread(boost::bind(
                &boost::asio::io_service::run,
               &io_service_
           ));
        }
    }

    ~thread_pool(){
        // Force all threads to return from io_service::run().
        io_service_.stop();

        // Suppress all exceptions.
        try
        {
            threads_.join_all();
        }
        catch (const std::exception&) {}

        delete[] apMatrices;
        delete[] randGens;
        delete[] randGens2;
        delete[] resourceSetAvailable;
        delete[] intersectionHandlingNoiseSources;
    }

private:
    // ***  M E T H O D S  *** //
    // *********************** //
	/**
	 * @brief Obtain the index of an available resource set
	 *
	 * Resource set is composed of:
	 *  apMatrices[index]
	 *  randGens[index]
	 *  randGens2[index]
	 *
	 * @return Available resource set index
	 */
	inline int getAvailableResourceSetIndex(){
	    for(size_t i = 0 ; i  < pool_size ; i++){
	        if(resourceSetAvailable[i]) return i;
	    }
	    return -1;
	}


public:
    /**
     * @brief Obtain the thread pool size
     * @return Thread pool size
     * @see thread_pool::pool_size
     */
    std::size_t getPoolSize(){return pool_size;}


	/**
	 * @brief Run a task when there is an available thread for it
	 */
	template < typename Task > void run_task(Task task){
		boost::unique_lock< boost::mutex > lock(mutex_);

		// If no threads are available, then wait for a thread to finish.
		if (0 == available_){
		    cond_.wait(lock);
		}

		// Decrement count, indicating thread is no longer available.
		--available_;

		// Get resource set index
		int resourceIdx = getAvailableResourceSetIndex();
		resourceSetAvailable[resourceIdx] = false;

        // Post a wrapped task into the queue.
		io_service_.post(
		    boost::bind(
		        &thread_pool::wrap_task,
		        this,
			    boost::function<
			        void(
			            std::vector<std::vector<double>>&,
			            RandomnessGenerator<double>&,
                        RandomnessGenerator<double>&,
                        NoiseSource<double>&
                    )
                >(task),
                resourceIdx
            )
        );
	}

	/**
	 * @brief Lock until all pending threads have finished
	 */
	void join(){
        boost::unique_lock<boost::mutex> lock(mutex_);
        while(available_ < pool_size){
            cond_.wait(lock);
        }
	}

private:
	/**
	 * @brief Wrap a task so that available threads count can be increased
	 *  once provided task has been completed
	 * @param task Task to be wrapped
	 * @param resourceIdx Resource index associated with the task. It is
	 *  necessary to release associated resources so other tasks can use
	 *  them later
	 */
	void wrap_task(
	    boost::function<void(
            std::vector<std::vector<double>>&,
            RandomnessGenerator<double>&,
            RandomnessGenerator<double>&,
            NoiseSource<double>&
        )> &task,
	    int resourceIdx
    ){
		// Run the user supplied task.
		try
		{
			task(
			    apMatrices[resourceIdx],
			    randGens[resourceIdx],
			    randGens2[resourceIdx],
			    intersectionHandlingNoiseSources[resourceIdx]
            );
		}
		// Suppress all exceptions.
		catch (const std::exception &e) {
			std::stringstream ss;
			ss << "thread_pool::wrap_task EXCEPTION: " << e.what();
			logging::WARN(ss.str());
		}

		// Task has finished, so increment count of available threads.
		boost::unique_lock< boost::mutex > lock(mutex_);
		++available_;
		resourceSetAvailable[resourceIdx] = true;
		cond_.notify_one();
	}
};
