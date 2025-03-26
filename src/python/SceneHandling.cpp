#include <KDTreeFactoryMaker.h>
#include <SceneHandling.h>
#include <SpectralLibrary.h>
#include <WavefrontObjFileLoader.h>
#include <GeoTiffFileLoader.h>
#include <XYZPointCloudFileLoader.h>


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
    glm::dvec3 defaultNormal
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

    loader.setAssetsDir(assetsPath);
   
    std::shared_ptr<ScenePart> sp(loader.run());
    for (auto p : sp->mPrimitives)
    p->part = sp;

    // Object lifetime caveat! Settings primsOut to nullptr will prevent the
    // loader destructor from deleting the primitives.
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
