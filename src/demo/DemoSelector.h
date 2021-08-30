#ifdef PCL_BINDING
#pragma once

#include <memory>
#include <string>

namespace HeliosDemos{

using std::shared_ptr;
using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Simple class to handle demo selection
 */
class DemoSelector{
private:
    // ***  SINGLETON: Instance  *** //
    // ***************************** //
    /**
     * @brief Singleton instance of demo selector
     * @see DemoSelector::getInstance
     */
    static shared_ptr<DemoSelector> ds;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for demo selector
     */
    DemoSelector() = default;
public:
    virtual ~DemoSelector() {};
    // ***  SINGLETON: Getter  *** //
    // *************************** //
    /**
     * @brief Singleton getter for the demo selector instance
     * @return Demo selector singleton instance
     * @see DemoSelector::ds
     */
    static shared_ptr<DemoSelector> getInstance();

    // ***  DEMO SELECTION METHOD  *** //
    // ******************************* //
    /**
     * @brief Run the demo with given name if any
     * @param name Name of the demo to be run
     * @param surveyPath Path to the survey to be used by the demo, if any.
     *  When there is no survey, an empty string should be passed
     */
    void select(
        string const name,
        string const surveyPath="",
        string const assetsPath=""
    );
};
}
#endif