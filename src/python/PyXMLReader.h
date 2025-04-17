#include <StaticScene.h>
#include <Survey.h>

#include <memory>
#include <string>
#include <vector>


std::shared_ptr<Survey> readSurveyFromXml(
    std::string surveyPath,
    std::vector<std::string> assetsPath,
    bool legNoiseDisabled
);

std::shared_ptr<Scanner> readScannerFromXml(
    std::string scannerPath,
    std::vector<std::string> assetsPath,
    std::string scannerId
);

std::shared_ptr<Platform> readPlatformFromXml(
    std::string platformPath,
    std::vector<std::string> assetsPath,
    std::string platformId
);

std::shared_ptr<Scene> readSceneFromXml(
    std::string filePath,
    std::vector<std::string> assetsPath,
    bool legNoiseDisabled
);

std::shared_ptr<ScenePart> readScenePartFromXml(
    std::string filePath,
    std::vector<std::string> assetsPath,
    int id
);

void finalizeStaticScene(
    std::shared_ptr<StaticScene>,
    int kdtFactoryType,
    int kdtNumJobs,
    int kdtGeomJobs,
    int kdtSAHLossNodes
);
