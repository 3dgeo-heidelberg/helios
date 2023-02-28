#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief A time watcher can be used to perform and report time measures
 */
class TimeWatcher{
public:
    // *** CONSTRUCTOR *** //
    // ******************* //
    /**
     * @brief Instantiate a time watcher
     */
    TimeWatcher();

    // *** PUBLIC METHODS *** //
    // ********************** //
    /**
     * @brief Start the time watcher, which sets the starting point for a time
     *  measure
     * @see TimeWatcher::startIfNull
     */
    void start();
    /**
     * @brief Start the time watcher but only if it has not been started
     *  before. Previous start might come either from TimeWatcher::start or
     *  from TimeWatcher::startIfNull
     * @see TimeWatcher::start
     */
    void startIfNull();
    /**
     * @brief Release stored start so it is null.
     *
     * For instance, following piece of code will only set the start time
     *  at first TimeWatcher::startIfNull call
     * \code
     *  TimeWatcher tw;
     *  tw.startIfNull();
     *  // Do stuff here
     *  tw.startIfNull();
     * \endcode
     *
     * However, following piece of code will set the start time at second
     *  TimeWatcher::startIfNull too
     * \code
     *  TimeWatcher tw;
     *  tw.startIfNull();
     *  // Do some stuff
     *  tw.releaseStart();
     *  // Do more stuff
     *  tw.startIfNull();
     * \endcode
     */
    void releaseStart();
    /**
     * @brief Check if time watcher has been started
     * @return True if time watcher has been started, false otherwise
     */
    bool hasStarted() const;
    /**
     * @brief Stop the time watcher, which sets the ending point for a time
     *  measure
     */
    void stop();
    /**
     * @brief Save the start time to local cache
     * @see TimeWatcher::tStart
     * @see TimeWatcher::loadStart
     */
    void saveStart();
    /**
     * @brief Load the start time from local cache
     * @see TimeWatcher::tStart
     * @see TimeWatcher::saveStart
     */
    void loadStart();
    /**
     * @brief Obtain the elapsed time as the difference between the last
     *  start() and the last stop() invocations.
     * @return Elapsed time
     * @see TimeWatcher::start
     * @see TimeWatcher::stop
     */
    std::shared_ptr<std::chrono::high_resolution_clock::duration>
    getElapsedTime();
    /**
     * @brief Obtain the elapsed time as the real number of seconds
     * @return Elapsed time in real seconds
     */
    double getElapsedDecimalSeconds();
    /**
     * @brief Obtain the elapsed time as the integer number of seconds
     * @return Elapsed time in integer seconds
     */
    long getElapsedSeconds();
    /**
     * @brief Obtain the elapsed time as the integer number of milliseconds
     * @return Elapsed time in integer milliseconds
     */
    long getElapsedMillis();
    /**
     * @brief Obtain the elapsed time as the integer number of nanoseconds
     * @return Elapsed time in integer nanoseconds
     */
    long getElapsedNanos();
    /**
     * @brief Obtain the elapsed time as a string with format "HH:MM:SS"
     * @return Elapsed time as "HH:MM:SS" string
     */
    std::string getElapsedFormat();
    /**
     * @brief Report elapsed seconds through specified output stream.
     * @param msg Message to be shown by the report. By default
     *  "Total elapsed seconds: "
     */
    void reportSeconds(
        std::string msg = "Total elapsed seconds: "
    );

    /**
     * @brief Report elapsed seconds through specified output stream.
     * @param msg Message to be shown by the report. By default
     *  "Total elapsed millisecconds: "
     */
    void reportMillis(
        std::string msg = "Total elapsed milliseconds: "
    );

    /**
     * @brief Report elapsed time through specified output stream using
     *  "HH:MM:SS" format.
     * @param msg Message to be shown by the report. By default
     *  "Total elapsed time: "
     */
    void reportFormat(
        std::string msg = "Total elapsed time: "
    );

    /**
     * @brief Synchronizes start, end and saved start times with those of given
     *  source
     * @param source Time watcher to be synchronized with
     */
    void synchronize(TimeWatcher const &source);
private:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Start time point
     *
     * @see TimeWatcher::start
     * @see TimeWatcher::savedStart
     */
    std::unique_ptr<std::chrono::high_resolution_clock::time_point> tStart;
    /**
     * @brief End time point
     *
     * @see TimeWatcher::stop
     */
    std::unique_ptr<std::chrono::high_resolution_clock::time_point> tEnd;

    // ***  CACHE  *** //
    // *************** //
    /**
     * @brief Used to store a temporary copy of current tStart
     * @see TimeWatcher::tStart
     * @see TimeWatcher::saveStart
     * @see TimeWatcher::loadStart
     */
    std::unique_ptr<std::chrono::high_resolution_clock::time_point> savedStart;

    // *** PRIVATE METHODS *** //
    // *********************** //
    /**
     * @brief Check if the time watcher has null start or end time points
     *
     * @return FALSE if the time watcher does not have neither a null start
     *  nor end time point. TRUE otherwise.
     */
    bool hasNulls();
    /**
     * @brief Synchronize time point \f$p\f$ with time point \f$q\f$
     * @param p Time point to be synchronized
     * @param q Time point to synchronize with
     */
    void syncTimePoints(
        std::unique_ptr<std::chrono::high_resolution_clock::time_point>
            &p,
        std::unique_ptr<std::chrono::high_resolution_clock::time_point> const
            &q
    );
};