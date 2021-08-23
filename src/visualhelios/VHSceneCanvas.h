#ifdef PCL_BINDING

#include <visualhelios/VHNormalsCanvas.h>
#include <visualhelios/adapters/VHDynSceneAdapter.h>
#include <visualhelios/adapters/VHDynObjectXYZRGBAdapter.h>

#include <memory>

namespace visualhelios{

using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Visual Helios Scene Canvas is a class which supports rendering
 *  a dynamic scene
 */
class VHSceneCanvas : public VHNormalsCanvas{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The dynamic scene that must be rendered by
     */
    shared_ptr<VHDynSceneAdapter<
        VHStaticObjectXYZRGBAdapter, VHDynObjectXYZRGBAdapter>
    > dynScene;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for the visual Helios scene canvas
     * @see visualhelios::VHNormalsCanvas::VHNormalsCanvas
     */
    VHSceneCanvas(DynScene &ds) :
        VHSceneCanvas(ds, "Visual Helios scene canvas")
    {}
    /**
     * @brief Constructor for the visual Helios scene canvas which allows for
     *  title specification, flags configuration and normal magnitude
     *  specification
     * @see VHNormalsCanvas(string const, bool const, bool const, bool const, float const)
     */
    VHSceneCanvas(
        DynScene &ds,
        string const title,
        bool const normalsKeyboardCallbackEnabled=true,
        bool const normalsUsageTextEnabled=true,
        bool const renderingNormals=true,
        float const normalMagnitude=1.0
    );
    virtual ~VHSceneCanvas() = default;

    // ***  CANVAS METHODS  *** //
    // ************************ //
    /**
     * @see VHNormalsCanvas::configure
     */
    void configure() override;
    /**
     * @see VHNormalsCanvas::start
     */
    void start() override;
    /**
     * @see VHNormalsCanvas::update
     */
    void update() override;

    // ***  NORMALS RENDERING METHODS  ***  //
    // ************************************ //
    /**
     * @brief Render normals for each primitive of given static object
     * @see visualhelios::VHNormalsCanvas::renderNormals
     */
    void renderNormals(VHStaticObjectAdapter & staticObj) override;
    /**
     * @brief Remove all rendered normals
     * @see visualhelios::VHNormalsCanvas::unrenderAllNormals
     */
    void unrenderAllNormals() override;

};

}

#endif