#ifdef PCL_BINDING
#pragma once

#include <demo/SurveyDemo.h>
#include <visualhelios/VHSceneCanvas.h>
#include <visualhelios/adapters/VHDynObjectXYZAdapter.h>
#include <Survey.h>

#include <string>

namespace HeliosDemos{

using visualhelios::VHSceneCanvas;
using visualhelios::VHDynObjectXYZAdapter;

using std::string;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Dynamic scene demo
 *
 * This demo implements the rendering of a given dynamic scene
 */
class DynamicSceneDemo : public SurveyDemo{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Dynamic scene demo constructor
     */
    DynamicSceneDemo(string const surveyPath, string const assetsPath) :
        SurveyDemo("Dynamic scene demo", surveyPath, assetsPath)
    {}
    virtual ~DynamicSceneDemo() = default;

    // ***   R U N   *** //
    // ***************** //
    /**
     * @see BaseDemo::run
     */
    void run() override;

    // ***   U T I L S   *** //
    // ********************* //
    /**
     * @brief Validate survey and assets path. If any of them is not valid a
     *  proper exception is thrown
     */
    void validatePaths();
    /**
     * @brief Load survey containing the scene to be rendered
     * @return Loaded survey containing the scene to be rendered
     */
    shared_ptr<Survey> loadSurvey();
    /**
     * @brief Build a visual Helios scene canvas from given survey.
     *
     * If the survey contains a dynamic scene it will be used. If it contains
     *  a basic scene, it will be wrapped by a dynamic scene so it can be
     *  rendered too. Notice however that, in the last case, no dynamic
     *  behavior will take place at all as the scene itself is not really
     *  dynamic.
     *
     * @param survey Survey which must contain a valid scene to build
     *  the visual Helios scene canvas
     * @return Built visual Helios scene canvas
     */
    shared_ptr<VHSceneCanvas> buildCanvas(shared_ptr<Survey> survey);
};

}

#endif