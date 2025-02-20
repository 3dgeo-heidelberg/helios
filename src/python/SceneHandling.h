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

void rotateScenePart(std::shared_ptr<ScenePart> sp, Rotation rotation);

void scaleScenePart(std::shared_ptr<ScenePart> sp, double scaleFactor);

void translateScenePart(std::shared_ptr<ScenePart> sp, glm::dvec3 offset);
