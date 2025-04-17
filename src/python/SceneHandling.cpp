#include <KDTreeFactoryMaker.h>
#include <SceneHandling.h>
#include <SpectralLibrary.h>
#include <WavefrontObjFileLoader.h>
#include <GeoTiffFileLoader.h>
#include <XYZPointCloudFileLoader.h>
#include <DetailedVoxelLoader.h>
#include <SerialSceneWrapper.h>


std::shared_ptr<KDTreeFactory> makeKDTreeFactory(int kdtFactoryType, int kdtNumJobs, int kdtGeomJobs, int kdtSAHLossNodes){
    if(kdtFactoryType == 1){ // Simple
        if(kdtNumJobs > 1){
            return KDTreeFactoryMaker::makeSimpleMultiThread(
                kdtNumJobs, kdtGeomJobs
            );
        }
        return KDTreeFactoryMaker::makeSimple();
    }
    else if(kdtFactoryType == 2){ // SAH
        if(kdtNumJobs > 1){
            return KDTreeFactoryMaker::makeSAHMultiThread(
                kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs
            );
        }
        return KDTreeFactoryMaker::makeSAH(kdtSAHLossNodes);
    }
    else if(kdtFactoryType == 3){ // Axis SAH
        if(kdtNumJobs > 1){
            return KDTreeFactoryMaker::makeAxisSAHMultiThread(
                kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs
            );
        }
        return KDTreeFactoryMaker::makeAxisSAH(kdtSAHLossNodes);
    }
    
    if(kdtNumJobs > 1){
        return KDTreeFactoryMaker::makeFastSAHMultiThread(
            kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs
        );
    }
    return KDTreeFactoryMaker::makeFastSAH(kdtSAHLossNodes);
}

void finalizeStaticScene(std::shared_ptr<StaticScene> scene, int kdtFactoryType, int kdtNumJobs, int kdtGeomJobs, int kdtSAHLossNodes) {
    // Loop over all scene parts and perform their final processing
    for (auto& sp : scene->parts) {
        // Append as a static object
        scene->appendStaticObject(sp);

        // Add scene part primitives to the scene
        scene->primitives.insert(
            scene->primitives.end(),
            sp->mPrimitives.begin(),
            sp->mPrimitives.end()
        );
    }

    // Call scene finalization
    if (!scene->finalizeLoading()) {
        throw std::runtime_error("Finalizing the scene failed.");
    }

    // Build KDGroveFactory
    scene->setKDGroveFactory(std::make_shared<KDGroveFactory>(makeKDTreeFactory(kdtFactoryType, kdtNumJobs, kdtGeomJobs, kdtSAHLossNodes)));
    scene->buildKDGroveWithLog();
}


void invalidateStaticScene(std::shared_ptr<StaticScene> scene) {
    scene->clearStaticObjects();
    scene->setKDGroveFactory(nullptr);
    scene->primitives.clear();
}


void setSceneReflectances(std::shared_ptr<StaticScene> scene, std::vector<std::string> assetsPath, float wavelength) {
    SpectralLibrary spectralLibrary(wavelength, assetsPath, "spectra");
    spectralLibrary.readReflectances();
    spectralLibrary.setReflectances(scene.get());
    scene->setDefaultReflectance(spectralLibrary.getDefaultReflectance());
}


std::shared_ptr<ScenePart> readObjScenePart(
    std::string filePath,
    std::vector<std::string> assetsPath,
    std::string upaxis
){
    WavefrontObjFileLoader loader;
    loader.params["filepath"] = filePath;
    loader.params["up"] = upaxis;
    loader.setAssetsDir(assetsPath);
    std::shared_ptr<ScenePart> sp(loader.run());

    // Connect all primitives to their scene part
    for (auto p : sp->mPrimitives)
        p->part = sp;

    // Object lifetime caveat! Settings primsOut to nullptr will prevent the
    // loader destructor from deleting the primitives.
    loader.primsOut = nullptr;

    return sp;
}

std::shared_ptr<ScenePart> readTiffScenePart(
    std::string filePath
){

    GeoTiffFileLoader loader;
    loader.params["filepath"] = filePath;
    std::shared_ptr<ScenePart> sp(loader.run());

    // Connect all primitives to their scene part
    for (auto p : sp->mPrimitives)
        p->part = sp;

    // Object lifetime caveat! Settings primsOut to nullptr will prevent the
    // loader destructor from deleting the primitives.
    loader.primsOut = nullptr;

    return sp;
}

std::shared_ptr<ScenePart> readXYZScenePart(
    std::string filePath,
    std::vector<std::string> assetsPath,
    std::string separator,
    double voxelSize,
    double maxColorValue,
    glm::dvec3 defaultNormal,
    bool sparse,
    int estimate_normals,
    int normalXIndex,
    int normalYIndex,
    int normalZIndex,
    int rgbRIndex,
    int rgbGIndex,
    int rgbBIndex,
    bool snapNeighborNormal
) {
   
    XYZPointCloudFileLoader loader;
    loader.params["filepath"] = filePath;
    loader.params["separator"] = separator;
    loader.params["voxelSize"] = voxelSize;
    if (maxColorValue != 0.0)
        loader.params["maxColorValue"] = maxColorValue;
    
    if ( defaultNormal.x != std::numeric_limits<double>::max() &&
         defaultNormal.y != std::numeric_limits<double>::max() &&
         defaultNormal.z != std::numeric_limits<double>::max() ){
        loader.params["defaultNormal"] = defaultNormal;}
    
    if (sparse)
        loader.params["sparse"] = sparse;

    if (estimate_normals != 0)
        loader.params["estimateNormals"] = estimate_normals;
    
    if (normalXIndex != 3){
        loader.params["normalXIndex"] = normalXIndex;
        loader.params["normalYIndex"] = normalYIndex;
        loader.params["normalZIndex"] = normalZIndex;
    }

    if (rgbRIndex != 6){
        loader.params["rgbRIndex"] = rgbRIndex;
        loader.params["rgbGIndex"] = rgbGIndex;
        loader.params["rgbBIndex"] = rgbBIndex;
    }

    if (snapNeighborNormal)
        loader.params["snapNeighborNormal"] = snapNeighborNormal;

    loader.setAssetsDir(assetsPath);
   
    std::shared_ptr<ScenePart> sp(loader.run());
    for (auto p : sp->mPrimitives)
    p->part = sp;

    // Object lifetime caveat! Settings primsOut to nullptr will prevent the
    // loader destructor from deleting the primitives.
    loader.primsOut = nullptr;

    return sp;
}

std::shared_ptr<ScenePart> readVoxScenePart(
    std::string filePath,
    std::vector<std::string> assetsPath,
    std::string intersectionMode,
    double intersectionArgument,
    bool randomShift,
    std::string ladlutPath
){
    if (intersectionMode == "fixed" && intersectionArgument != 0.0) {
        throw std::invalid_argument("'intersectionArgument' must not be provided when 'intersectionMode' is 'fixed'.");
    }
    DetailedVoxelLoader loader;
    loader.params["filepath"] = filePath;
    loader.params["intersectionMode"] = intersectionMode;
    if (intersectionMode == "scaled") 
        loader.params["intersectionArgument"] = intersectionArgument;
    if (randomShift)
        loader.params["randomShift"] = randomShift;
    if (!ladlutPath.empty())
        loader.params["ladlut"] = ladlutPath;

    loader.setAssetsDir(assetsPath);
    std::shared_ptr<ScenePart> sp(loader.run());
    
    for (auto p : sp->mPrimitives)
        p->part = sp;

    loader.primsOut = nullptr;

    return sp;
}

void rotateScenePart(std::shared_ptr<ScenePart> sp, Rotation rotation) {
    for (auto p : sp->mPrimitives)
        p->rotate(rotation);
}


void scaleScenePart(std::shared_ptr<ScenePart> sp, double scaleFactor) {
    for (auto p : sp->mPrimitives)
        p->scale(scaleFactor);
}


void translateScenePart(std::shared_ptr<ScenePart> sp, glm::dvec3 offset) {
    for (auto p : sp->mPrimitives)
        p->translate(offset);
}

void writeSceneToBinary(
    const std::string& filename,
    std::shared_ptr<Scene> scene,
    bool isDynScene
) {


    SerialSceneWrapper::SceneType sceneType;
    if (isDynScene) {
        sceneType = SerialSceneWrapper::SceneType::DYNAMIC_SCENE;
    } else {
        sceneType = SerialSceneWrapper::SceneType::STATIC_SCENE;
    }
    SerialSceneWrapper(sceneType, scene.get()).writeScene(filename);
}

std::shared_ptr<Scene> readSceneFromBinary(
    const std::string& filename
) {
    fs::path filePath = fs::path(filename);

    if (!fs::is_regular_file(filePath)) {
        throw std::runtime_error("Binary Scene file does not exist: " + filePath.string());
    }

    SerialSceneWrapper *ssw = SerialSceneWrapper::readScene(filePath.string());
    std::shared_ptr<Scene> scene = std::shared_ptr<Scene>(ssw->getScene());
    delete ssw;

    return scene;
}