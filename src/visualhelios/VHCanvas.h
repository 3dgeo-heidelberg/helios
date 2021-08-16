#ifdef PCL_BINDING

#pragma once

#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace visualhelios{

using std::string;

using pcl::visualization::PCLVisualizer;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Visual Helios Canvas is a class which provides the base mechanisms
 *  to implement Helios visualizations.
 *
 *
 * The visual helios canvas is based on PCL and VTK libraries. Its workflow can
 *  be triggered by the user calling the show method and it is as follows:
 *  configure -> start -> update while non stopped -> onstop
 *
 */
class VHCanvas{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The title of the visual Helios canvas
     */
    string const title;
    /**
     * @brief The PCL visualizer which is used to render graphics
     */
    PCLVisualizer::Ptr viewer;
    /**
     * @brief How many milliseconds must elapsed between canvas updates
     */
    int timeBetweenUpdates;
    /**
     * @brief Force redraw even when it is not required if true. Try to avoid
     *  unnecessary redraws if false
     */
    bool forceRedraw;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for the visual helios canvas
     */
    VHCanvas() : VHCanvas("Visual Helios canvas") {}
    /**
     * @brief Constructor for the visual helios canvas which allows for
     *  title specification
     * @param title Title for the visualizer
     */
    VHCanvas(string const title);
    virtual ~VHCanvas() = default;

protected:
    // ***  CANVAS METHODS  *** //
    // ************************ //
    /**
     * @brief Configure method where visualizer configuration must be
     *  implemented.
     */
    virtual void configure();
    /**
     * @brief Start method which initializes the visualization.
     */
    virtual void start();
    /**
     * @brief Update method which handles graphics updating over time.
     */
    virtual void update();
    /**
     * @brief Method to handle the behavior of the canvas after visualization
     *  has finished
     */
    virtual void onStop();
public:
    /**
     * @brief Make the visualization effective
     */
    virtual void show();

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain milliseconds between canvas updates
     * @return Milliseconds between canvas updates
     * @see VHCanvas::timeBetweenUpdates
     */
    inline int getTimeBetweenUpdates() const {return timeBetweenUpdates;}
    /**
     * @brief Set the milliseconds between canvas updates
     * @param timeBetweenUpdates How many milliseconds must elapse between
     *  canvas updates
     * @see VHCanvas::timeBetweenUpdates
     */
    inline void setTimeBetweenUpdates(int const timeBetweenUpdates)
    {this->timeBetweenUpdates = timeBetweenUpdates;}
    /**
     * @brief Check if force redraw is enabled or not
     * @return True if force redraw is enabled, false otherwise
     * @see VHCanvas::forceRedraw
     */
    inline bool isForceRedraw() const {return forceRedraw;}
    /**
     * @brief Enable or disable force redraw
     * @param forceRedraw True to enable force redraw, false to disable it
     * @see VHCanvas::forceRedraw
     */
    inline void setForceRedraw(bool const forceRedraw)
    {this->forceRedraw = forceRedraw;}
    /**
     * @brief Obtain the visual helios canvas title
     * @return Visual helios canvas title
     */
    inline string const & getTitle() const {return title;}
};
}

#endif