#ifdef PCL_BINDING

#include <memory>
#include <vector>

#include <visualhelios/VHCanvas.h>
#include <visualhelios/adapters/VHDynObjectXYZRGBAdapter.h>

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
    vector<shared_ptr<VHDynObjectXYZRGBAdapter>> dynObjs;
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
        vector<shared_ptr<VHDynObjectXYZRGBAdapter>>
    )> dynamicUpdateFunction;

    /**
     * @brief Specify if the simple canvas must render normals (true) or not
     *  (false)
     */
    bool renderingNormals;
    /**
     * @brief Specify the magnitude of normal vector for visualization
     */
    float normalMagnitude;

    /**
     * @brief Control whether an update is needed even when dynamic objects
     *  themselves have not been updated (true) or not (false)
     */
    bool needUpdate;

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
     * @see VHCanvas::configure
     */
    void configure() override;
    /**
     * @see VHCanvas::start
     */
    void start() override;
    /**
     * @see VHCanvas::update
     */
    void update() override;

    // ***  UTIL METHODS  ***  //
    // ********************** //
    /**
     * @brief Render normals for each primitive of given dynamic object
     */
    void renderNormals(VHDynObjectXYZRGBAdapter & dynObj);
    /**
     * @brief Remove all rendered normals
     */
    void unrenderAllNormals();

public:
    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Append a dynamic object to the simple canvas
     * @param dynObj Dynamic object to be appended
     * @see VHSimpleCanvas::dynObjs
     */
    inline void appendDynObj(shared_ptr<VHDynObjectXYZRGBAdapter> dynObj)
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
    inline void setDynObj(
        size_t index,
        shared_ptr<VHDynObjectXYZRGBAdapter> dynObj
    )
    {dynObjs[index] = dynObj;}
    /**
     * @brief Remove all dynamic objects from simple canvas
     */
    inline void clearDynObjs()
    {dynObjs.clear();}
    /**
     * @brief Set the dynamic update function
     * @param dynamicUpdateFunction  The dynamic update function
     * @see visualhelios::VHSimpleCanvas::dynamicUpdateFunction
     */
    inline void setDynamicUpdateFunction(
        std::function<void(
            vector<shared_ptr<VHDynObjectXYZRGBAdapter>>
        )> const dynamicUpdateFunction
    ){this->dynamicUpdateFunction = dynamicUpdateFunction;}
    /**
     * @brief Check whether the simple canvas is rendering normals or not
     * @return True if simple canvas is rendering normals, false otherwise
     * @see visualhelios::VHSimpleCanvas::renderingNormals
     */
    inline bool isRenderingNormals() const {return renderingNormals;}
    /**
     * @brief Enable or disable normals rendering
     * @param renderingNormals True to enable rendering normals, false
     *  to disable it
     * @see visualhelios::VHSimpleCanvas::renderingNormals
     */
    inline void setRenderingNormals(bool const renderingNormals)
    {this->renderingNormals = renderingNormals;}


};

}

#endif