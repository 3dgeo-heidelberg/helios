#include <KDGroveRaycaster.h>

// ***  RAYCASTING METHODS  *** //
// **************************** //
std::map<double, Primitive*> KDGroveRaycaster::searchAll(
    glm::dvec3 rayOrigin,
    glm::dvec3 rayDir,
    double tmin,
    double tmax,
    bool groundOnly
){
    std::map<double, Primitive *> out;
    size_t const m = grove->getNumTrees();
    for(size_t i = 0 ; i < m ; ++i){
        std::map<double, Primitive *> partial =
            grove->getTreeShared(i)->searchAll(
                rayOrigin, rayDir, tmin, tmax, groundOnly
            );
        out.insert(partial.begin(), partial.end());
    }
    return out;
}
RaySceneIntersection * KDGroveRaycaster::search(
    glm::dvec3 rayOrigin,
    glm::dvec3 rayDir,
    double tmin,
    double tmax,
    bool groundOnly
){
    std::map<double, Primitive *> out;
    size_t const m = grove->getNumTrees();
    RaySceneIntersection *bestRSI = nullptr, *rsi = nullptr;
    for(size_t i = 0 ; i < m ; ++i){
        rsi = grove->getTreeShared(i)->search(
            rayOrigin, rayDir, tmin, tmax, groundOnly
        );
        if(
            bestRSI==nullptr ||
            (rsi!=nullptr && rsi->hitDistance < bestRSI->hitDistance)
        ){
            bestRSI = rsi;
        }
    }
    return bestRSI;
}

// ***  KDGROVE RELATED METHODS  *** //
// ********************************* //
std::shared_ptr<KDGroveRaycaster> KDGroveRaycaster::makeTemporalClone() const{
    return std::make_shared<KDGroveRaycaster>(grove->makeTemporalClone());
}
