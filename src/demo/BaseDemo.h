#ifdef PCL_BINDING
#pragma once

#include <iostream>
#include <string>

namespace HeliosDemos {

using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 *
 * @brief BaseDemo class
 *
 * Can be overridden to implement new demos.
 *
 * NOTICE in order for a demo to be runnable it must override the run method
 * to implement the demo behavior
 */
class BaseDemo{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The name for the demo
     */
    string const name;
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Base demo constructor
     * @param name Name for the demo
     */
    BaseDemo(string const name) : name(name) {};
    virtual ~BaseDemo() = default;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the demo name
     * @return Demo name
     */
    string getName() {return name;}

    // ***  R U N  *** //
    // *************** //
    /**
     * @brief Run the demo itself
     *
     * Only demos implementing a run method will be runnable
     */
    virtual void run() = 0;
};
}
#endif