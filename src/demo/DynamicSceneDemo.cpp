#ifdef PCL_BINDING

#include <demo/DynamicSceneDemo.h>
#include <XmlSurveyLoader.h>
#include <HeliosException.h>

using HeliosDemos::DynamicSceneDemo;
using visualhelios::VHSceneCanvas;
using std::make_shared;
using std::static_pointer_cast;


// ***   R U N   *** //
// ***************** //
void DynamicSceneDemo::run(){
    std::cout << "RUNNING DYNAMIC SCENE DEMO ..." << std::endl;

    // Validate paths
    validatePaths();

    // Load survey
    shared_ptr<Survey> survey = loadSurvey();

    // Build dynamic scene canvas
    shared_ptr<VHSceneCanvas> canvas = buildCanvas(survey);

    // Render canvas
    canvas->show();

    std::cout << "FINISHED DYNAMIC SCENE DEMO!" << std::endl;
}

// ***   U T I L S   *** //
// ********************* //
void DynamicSceneDemo::validatePaths(){
    if(!validateSurveyPath()){
        std::stringstream ss;
        ss  << "DynamicSceneDemo could not validate survey path:\n\t\""
            << surveyPath << "\"";
        throw HeliosException(ss.str());
    }
    if(!validateAssetsPath()){
        std::stringstream ss;
        ss  << "DynamicSceneDemo could not validate assets path:\n\t\""
            << assetsPath << "\"";
        throw HeliosException(ss.str());
    }
}
shared_ptr<Survey> DynamicSceneDemo::loadSurvey(){
    shared_ptr<XmlSurveyLoader> xmlreader(
        new XmlSurveyLoader(surveyPath, assetsPath)
    );
    shared_ptr<Survey> survey = xmlreader->load(true, true);
    if(survey == nullptr){
        std::stringstream ss;
        ss  << "DynamicSceneDemo failed to read scene from survey:\n\t\""
            << surveyPath << "\"";
        throw HeliosException(ss.str());
    }
    survey->scanner->setWriteWaveform(false);
    survey->scanner->setCalcEchowidth(false);
    survey->scanner->setFullWaveNoise(false);
    survey->scanner->setPlatformNoiseDisabled(false);
    survey->scanner->setFixedIncidenceAngle(false);
    survey->scanner->detector->lasOutput = false;
    survey->scanner->detector->las10 = false;
    survey->scanner->detector->zipOutput = false;
    survey->scanner->detector->lasScale = 1.0;
    return survey;
}

shared_ptr<VHSceneCanvas> DynamicSceneDemo::buildCanvas(
    shared_ptr<Survey> survey
){
    // Prepare canvas construction arguments
    string canvasTitle = "Dynamic scene demo";
    bool normalsKeyboardCallback = true;
    bool normalsUsageText = true;
    bool renderNormals = false;
    float normalMagnitude = 1.0;
    shared_ptr<VHSceneCanvas> canvas;

    // Assuming the scene is already dynamic, try to cast it
    try{
        DynScene &ds = dynamic_cast<DynScene &>(
            *survey->scanner->platform->scene
        );
        canvas = make_shared<VHSceneCanvas>(
            ds,
            canvasTitle,
            normalsKeyboardCallback,
            normalsUsageText,
            renderNormals,
            normalMagnitude
        );
    }
    // If scene was not dynamic, wrap it into a dynamic scene
    catch(std::bad_cast &bcex){
        std::cout   << "DynamicSceneDemo received a non dynamic scene but a "
                    << "basic one. In consequence, it was wrapped."
                    << std::endl;
        dsWrapper = make_shared<DynScene>(*static_pointer_cast<StaticScene>(
            survey->scanner->platform->scene
        ));
        canvas = make_shared<VHSceneCanvas>(
            *dsWrapper,
            canvasTitle,
            normalsKeyboardCallback,
            normalsUsageText,
            renderNormals,
            normalMagnitude
        );
    }

    // Configure canvas
    canvas->setTimeBetweenUpdates(20);

    // Return canvas
    return canvas;
}

#endif