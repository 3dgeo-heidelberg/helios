#pragma once

#include <string>
#include <iostream>
#include <iomanip>

namespace HeliosTests {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief BaseTest class
 *
 * Can be overridden to implement new tests.
 *
 * NOTICE in order for a test to be runnable it must override the run method
 * to implement the test behavior.
 */
class BaseTest {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The name for the test
     */
    std::string const name;
public:
    // ***  CONSTRUCTION  *** //
    // ********************** //
    /**
     * @brief Base test constructor
     * @param name Name for the test
     */
    BaseTest(std::string const &name) : name(name) {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the test name
     * @return Test name
     */
    std::string getName(){return name;}

    // ***  R U N  *** //
    // *************** //
    /**
     * @brief Test behavior.
     *
     * At the end it must report test status.
     *
     * Only tests implementing a run method will be runnable
     *
     * @return If test finished with successful status true must be returned.
     * Otherwise false will be returned.
     */
    virtual bool run() = 0;

    // ***  T E S T  *** //
    // ***************** //
    /**
     * @brief Another way to call test function
     *
     * @see HeliosTests::BaseTest::test()
     */
    void operator()(std::ostream &out=std::cout, bool color=true)
        {test(out, color);}
    /**
     * @brief Perform the test and output its final status
     * @param out Output stream used to output test final status
     * @param color True to enable coloring when reporting test status.
     *  False otherwise
     */
    void test(std::ostream &out=std::cout, bool color=true);
};

// *** CLASS IMPLEMENTATION *** //
// **************************** //
void BaseTest::test(std::ostream &out, bool color){
    // Do test
    bool status = run();

    // Report test status
    if(color) out << "\033[1m";
    out << "TEST ";
    if(color) out << "\033[0m";
    out << std::setw(52) << std::left << name.c_str() << " ";
    if(color) out << "\033[1m";
    out << "[";
    if(color){
        if(status) out << "\033[32m";
        else out << "\033[31m";
    }
    out << (status ? "PASSED" : "FAILED");
    if(color) out << "\033[0m\033[1m";
    out << "]";
    if(color) out << "\033[0m";
    out << std::endl;
}

}