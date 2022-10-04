#pragma once

#include "BaseTest.h"
#include <Survey.h>
#include <OscillatingMirrorBeamDeflector.h>
#include <HelicopterPlatform.h>
#include <FullWaveformPulseDetector.h>
#include <scanner/SingleScanner.h>

namespace HeliosTests{
/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Test survey copy
 */
class SurveyCopyTest : public BaseTest{
public:
    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief Survey copy test constructor
     */
    SurveyCopyTest() : BaseTest("Survey copy test"){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;
};

bool SurveyCopyTest::run(){
    // Build base Survey
    std::shared_ptr<Survey> survey = std::make_shared<Survey>();
    survey->name = "MySurvey";
    survey->numRuns = 1;
    survey->simSpeedFactor = 1;
    std::list<int> pulseFreqs;
    pulseFreqs.push_back(100);
    pulseFreqs.push_back(30);
    pulseFreqs.push_back(70);
    survey->scanner = std::make_shared<SingleScanner>(
        0.1,
        glm::dvec3(2.0, 3.0, 0.0),
        Rotation(0.0, 0.0, 0.0, 0.0, true),
        pulseFreqs,
        4.0,
        "MyScanner",
        80.5,
        3.0,
        0.9,
        0.7,
        0.8,
        100,
        false,
        false,
        false,
        true
    );
    survey->scanner->setScannerHead(std::make_shared<ScannerHead>(
        glm::dvec3(0.4, 0.7, 0.1), 0.067
    ));
    survey->scanner->setBeamDeflector(
        std::make_shared<OscillatingMirrorBeamDeflector>(
            3.141592,
            1400.5,
            70.8,
            1
        ));
    survey->scanner->platform = std::make_shared<HelicopterPlatform>();
    survey->scanner->setDetector(std::make_shared<FullWaveformPulseDetector>(
        survey->scanner,
        1.5,
        0.1
    ));
    survey->legs.push_back(std::make_shared<Leg>());
    survey->legs[0]->mPlatformSettings = std::make_shared<PlatformSettings>();
    survey->legs[0]->mPlatformSettings->onGround = false;
    survey->scanner->platform->scene = std::make_shared<Scene>();
    std::shared_ptr<Scene> baseScene = survey->scanner->platform->scene;
    baseScene->primitives.push_back(new Triangle(
        Vertex(), Vertex(), Vertex()
    ));
    baseScene->primitives[0]->part = std::make_shared<ScenePart>();
    baseScene->primitives[0]->part->mPrimitives.push_back(
        baseScene->primitives[0]);
    baseScene->primitives[0]->part->onRayIntersectionMode = "TRANSMITTIVE";
    baseScene->primitives.push_back(new DetailedVoxel(
        glm::dvec3(0.0, 0.0, 0.5),
        2.15,
        std::vector<int>({1,2}),
        std::vector<double>({0.1, 0.2, 0.3})
    ));
    baseScene->primitives[1]->material = std::make_shared<Material>();
    baseScene->primitives[1]->material->ka[0] = 1.1;
    baseScene->primitives[1]->material->ks[1] = 1.2;

    // Copy base Survey
    std::shared_ptr<Survey> copy = std::make_shared<Survey>(*survey);

    // Do some changes on copy
    copy->name = "CopiedSurvey";
    copy->numRuns = 0;
    copy->scanner->name = "CopiedScanner";
    Rotation &copyMRA =
        copy->scanner->getScannerHead()->getMountRelativeAttitudeByReference();
    copyMRA.setQ3(copyMRA.getQ3() + 0.1);
    copy->scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz += 1.0;
    copy->scanner->platform->cfg_device_relativeMountPosition.x += 0.01;
    HelicopterPlatform *hp =
        ((HelicopterPlatform *)copy->scanner->platform.get());
    glm::dvec3 & speedXy = hp->getSpeedXyByReference();
    speedXy.x += 0.1;
    Rotation & r = hp->getRotationByReference();
    r.setQ2(r.getQ2()+0.1);
    copy->scanner->getFWFSettings().minEchoWidth += 0.001;
    copy->legs[0]->mPlatformSettings->onGround = true;
    std::shared_ptr<Scene> copyScene = copy->scanner->platform->scene;
    copyScene->primitives[0]->getVertices()[0].pos.x += 0.1;
    copyScene->primitives[0]->part->onRayIntersectionArgument += 0.034;
    copyScene->primitives[1]->material->ks[1] += 0.1;
    DetailedVoxel *copyDv = (DetailedVoxel *) copyScene->primitives[1];
    (*copyDv)[1] += 0.1;

    // Validate copy
    if(copy->name == survey->name) return false;
    if(copy->numRuns == survey->numRuns) return false;
    if(copy->simSpeedFactor != survey->simSpeedFactor) return false;
    if(copy->scanner->name == survey->scanner->name) return false;
    if(copy->scanner->getNumTimeBins()!=survey->scanner->getNumTimeBins())
        return false;
    if(copy->scanner->isCalcEchowidth() != survey->scanner->isCalcEchowidth())
        return false;
    if(copy->scanner->getFWFSettings().minEchoWidth ==
            survey->scanner->getFWFSettings().minEchoWidth)
        return false;
    if(copy->scanner->getFWFSettings().apertureDiameter !=
       survey->scanner->getFWFSettings().apertureDiameter)
        return false;
    if(copy->scanner->getScannerHead()->getRotatePerSecMax() !=
            survey->scanner->getScannerHead()->getRotatePerSecMax())
        return false;
    Rotation &baseMRA = survey->scanner->getScannerHead()
        ->getMountRelativeAttitudeByReference();
    if(copyMRA.getQ0()!=baseMRA.getQ0() || copyMRA.getQ3()==baseMRA.getQ3())
        return false;
    if(copy->scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz !=
            survey->scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz)
        return false;
    if(copy->scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz ==
            survey->scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz)
        return false;
    if(copy->scanner->platform->cfg_device_relativeMountPosition.x ==
            survey->scanner->platform->cfg_device_relativeMountPosition.x)
        return false;
    if(copy->scanner->platform->cfg_device_relativeMountPosition.y !=
            survey->scanner->platform->cfg_device_relativeMountPosition.y)
        return false;
    HelicopterPlatform *copyHp =
        (HelicopterPlatform *) copy->scanner->platform.get();
    HelicopterPlatform *baseHp =
        (HelicopterPlatform *) survey->scanner->platform.get();
    glm::dvec3 &copySpeedXy = copyHp->getSpeedXyByReference();
    glm::dvec3 &baseSpeedXy = baseHp->getSpeedXyByReference();
    if(copySpeedXy.x == baseSpeedXy.x) return false;
    if(copySpeedXy.y != baseSpeedXy.y) return false;
    Rotation &copyRot = copyHp->getRotationByReference();
    Rotation &baseRot = baseHp->getRotationByReference();
    if(copyRot.getQ1() != baseRot.getQ1()) return false;
    if(copyRot.getQ2() == baseRot.getQ2()) return false;
    if(copy->legs[0]->mPlatformSettings->onGround ==
            survey->legs[0]->mPlatformSettings->onGround)
        return false;
    if(copy->legs[0]->mPlatformSettings->stopAndTurn !=
            survey->legs[0]->mPlatformSettings->stopAndTurn)
        return false;
    if(copyScene->primitives[0]->getVertices()[0].pos.x ==
            baseScene->primitives[0]->getVertices()[0].pos.x)
        return false;
    if(copyScene->primitives[0]->getVertices()[0].pos.y !=
            baseScene->primitives[0]->getVertices()[0].pos.y)
        return false;
    if(copyScene->primitives[0]->part->onRayIntersectionArgument ==
            baseScene->primitives[0]->part->onRayIntersectionArgument)
        return false;
    if(copyScene->primitives[0]->part->onRayIntersectionMode !=
            baseScene->primitives[0]->part->onRayIntersectionMode)
        return false;
    if(copyScene->primitives[1]->material->ka[0] !=
            baseScene->primitives[1]->material->ka[0])
        return false;
    if(copyScene->primitives[1]->material->ks[1] ==
            baseScene->primitives[1]->material->ks[1])
        return false;
    DetailedVoxel *baseDv = (DetailedVoxel *) baseScene->primitives[1];
    copyDv = (DetailedVoxel *) copyScene->primitives[1];
    if((*baseDv)[0] != (*copyDv)[0]) return false;
    if((*baseDv)[1] == (*copyDv)[1]) return false;

    return true;
}

}