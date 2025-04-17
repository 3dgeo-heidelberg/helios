#pragma once
#include <Rotation.h>
#include <StaticScene.h>

#include <memory>
#include <string>
#include <vector>

void invalidateStaticScene(std::shared_ptr<StaticScene> scene);

void setSceneReflectances(std::shared_ptr<StaticScene> scene, std::vector<std::string> assetsPath, float wavelength);

std::shared_ptr<ScenePart> readObjScenePart(
    std::string filePath,
    std::vector<std::string> assetsPath,
    std::string upaxis
);

std::shared_ptr<ScenePart> readTiffScenePart(
    std::string filePath
);

std::shared_ptr<ScenePart> readXYZScenePart(
    std::string filePath,
    std::vector<std::string> assetsPath,
    std::string separator,
    double voxelSize,
    double maxColorValue = 0.0,
    glm::dvec3 defaultNormal = glm::dvec3(
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()
    ),
    bool sparse = false,
    int estimate_normals = 0,
    int normalXIndex = 3,
    int normalYIndex = 4,
    int normalZIndex = 5,
    int rgbRIndex = 6,
    int rgbGIndex = 7,
    int rgbBIndex = 8,
    bool snapNeighborNormal = false
); 

std::shared_ptr<ScenePart> readVoxScenePart(
    std::string filePath,
    std::vector<std::string> assetsPath,
    std::string intersectionMode,
    double intersectionArgument,
    bool randomShift = false,
    std::string ladlutPath = "" 
);

void rotateScenePart(std::shared_ptr<ScenePart> sp, Rotation rotation);

void scaleScenePart(std::shared_ptr<ScenePart> sp, double scaleFactor);

void translateScenePart(std::shared_ptr<ScenePart> sp, glm::dvec3 offset);

void writeSceneToBinary(
    const std::string& writePath,
    std::shared_ptr<Scene> scene,
    bool isDynScene = false
);

std::shared_ptr<Scene> readSceneFromBinary(
    const std::string& readPath
);
