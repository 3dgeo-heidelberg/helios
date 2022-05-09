#ifdef PCL_BINDING

#include <demo/RaycastingDemo.h>
#include <XmlSurveyLoader.h>
#include <util/HeliosException.h>
#include <filems/facade/FMSFacade.h>
#include <filems/facade/FMSWriteFacade.h>
#include <filems/factory/FMSFacadeFactory.h>

#include <memory>

using HeliosDemos::RaycastingDemo;
using visualhelios::VHRaycastingCanvas;
using std::static_pointer_cast;

// ***  R U N  *** //
// *************** //
/**
 * @see DynamicSceneDemo::run
 */
void RaycastingDemo::run(){
    std::cout << "RUNNING RAYCASTING DEMO ..." << std::endl;

    // Validate paths
    validatePaths();

    // Load survey
    shared_ptr<Survey> survey = loadSurvey();

    // Build dynamic scene canvas
    shared_ptr<VHRaycastingCanvas> canvas = buildCanvas(survey);

    // Render canvas
    canvas->show();

    std::cout << "FINISHED RAYCASTING DEMO!" << std::endl;
}
// ***   U T I L S   *** //
// ********************* //
shared_ptr<VHRaycastingCanvas> RaycastingDemo::buildCanvas(
    shared_ptr<Survey> survey
){
    // Prepare canvas construction arguments
    string canvasTitle = "Raycasting demo";
    bool normalsKeyboardCallback = true;
    bool normalsUsageText = true;
    bool renderNormals = false;
    float normalMagnitude = 1.0;
    shared_ptr<VHRaycastingCanvas> canvas;

    // Assuming the scene is already dynamic, try to cast it
    try{
        DynScene &ds = dynamic_cast<DynScene &>(
            *survey->scanner->platform->scene
        );
        canvas = make_shared<VHRaycastingCanvas>(
            ds,
            *survey->scanner,
            *survey,
            canvasTitle,
            normalsKeyboardCallback,
            normalsUsageText,
            renderNormals,
            normalMagnitude
        );
    }
    // If scene was not dynamic, wrap it into a dynamic scene
    catch(std::bad_cast &bcex){
        std::cout   << "RaycastingDemo received a non dynamic scene but a "
                    << "basic one. In consequence, it was wrapped."
                    << std::endl;
        dsWrapper = make_shared<DynScene>(*static_pointer_cast<StaticScene>(
            survey->scanner->platform->scene
        ));
        canvas = make_shared<VHRaycastingCanvas>(
            *dsWrapper,
            *survey->scanner,
            *survey,
            canvasTitle,
            normalsKeyboardCallback,
            normalsUsageText,
            renderNormals,
            normalMagnitude
        );
    }

    // Configure canvas
    canvas->setTimeBetweenUpdates(5);

    // Return canvas
    return canvas;
}

#endif