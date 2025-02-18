#pragma once
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

void scaleScenePart(std::shared_ptr<ScenePart> sp, double scaleFactor);
