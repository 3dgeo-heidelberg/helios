#ifdef PCL_BINDING

#include <memory>
#include <vector>

#include <visualhelios/VHCanvas.h>
#include <visualhelios/adapters/VHDynObjectAdapter.h>

namespace visualhelios{

using std::shared_ptr;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Visual Helios Simple Canvas is a class which supports rendering
 *  polygon meshes which are updated over time
 */
class VHSimpleCanvas : public VHCanvas{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The dynamic objects that must be rendered by the simple canvas
     * @see VHSimpleCanvas::appendDynObj
     * @see VHSimpleCanvas::getDynObj
     * @see VHSimpleCanvas::setDynObj
     * @see VHSimpleCanvas::clearDynObjs
     */
    vector<shared_ptr<VHDynObjectAdapter>> dynObjs;
    /**
     * @brief Function to define dynamic objects behavior before updating the
     *  canvas
     *
     * It can be used to populate the motion queues or any other required
     *  manipulation over dynamic objects.
     *
     * Simple canvas does not rely on components which update the dynamic
     *  object outside the canvas scope, so any implementation to define the
     *  behavior of dynamic objects must be done through this ad-hoc function
     */
    std::function<void(
        vector<shared_ptr<VHDynObjectAdapter>>
    )> dynamicUpdateFunction;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for the visual helios simple canvas
     * @see visualhelios::VHCanvas::VHCanvas
     */
    VHSimpleCanvas() : VHSimpleCanvas("Visual Helios simple canvas") {}
    /**
     * @brief Constructor for the visual helios simple canvas which allows for
     *  title specification
     * @param title Title for the visualizer
     * @see visualhelios::VHCanvas::VHCanvas(string const)
     */
    VHSimpleCanvas(string const title);
    virtual ~VHSimpleCanvas() = default;

protected:
    // ***  CANVAS METHODS  *** //
    // ************************ //
    /**
     * @see VHCanvas::start
     */
    void start() override;
    /**
     * @see VHCanvas::update
     */
    void update() override;

public:
    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Append a dynamic object to the simple canvas
     * @param dynObj Dynamic object to be appended
     * @see VHSimpleCanvas::dynObjs
     */
    inline void appendDynObj(shared_ptr<VHDynObjectAdapter> dynObj)
    {dynObjs.push_back(dynObj);}
    /**
     * @brief Obtain a dynamic object from simple canvas
     * @param index Index of dynamic object to be obtained
     * @return Dynamic object from simple canvas
     */
    inline VHDynObjectAdapter const & getDynObj(size_t index) const
    {return *dynObjs[index];}
    /**
     * @brief Replace a dynamic object in simple canvas
     * @param index Index of dynamic object to be replaced
     * @param dynObj Dynamic object to replace with
     */
    inline void setDynObj(size_t index, shared_ptr<VHDynObjectAdapter> dynObj)
    {dynObjs[index] = dynObj;}
    /**
     * @brief Remove all dynamic objects from simple canvas
     */
    inline void clearDynObjs()
    {dynObjs.clear();}
    /**
     * @brief Set the dynamic update function
     * @param dynamicUpdateFunction
     * @see visualhelios::VHSimpleCanvas::DynamicUpdateFunction
     */
    inline void setDynamicUpdateFunction(
        std::function<void(
            vector<shared_ptr<VHDynObjectAdapter>>
        )> const dynamicUpdateFunction
    ){this->dynamicUpdateFunction = dynamicUpdateFunction;}

};

}

#endif