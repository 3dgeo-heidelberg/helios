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
     */
    void start();
    /**
     * @brief Stop the time watcher, which sets the ending point for a time
     *  measure
     */
    void stop();
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
private:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Start time point
     *
     * @see TimeWatcher::start
     */
    std::unique_ptr<std::chrono::high_resolution_clock::time_point> tStart;
    /**
     * @brief End time point
     *
     * @see TimeWatcher::stop
     */
    std::unique_ptr<std::chrono::high_resolution_clock::time_point> tEnd;

    // *** PRIVATE METHODS *** //
    // *********************** //
    /**
     * @brief Check if the time watcher has null start or end time points
     *
     * @return FALSE if the time watcher does not have neither a null start
     *  nor end time point. TRUE otherwise.
     */
    bool hasNulls();
};