#include <DynFullWaveformPulseRunnable.h>

// ***  ASSISTANCE METHODS  *** //
// **************************** //
shared_ptr<RaySceneIntersection>
DynFullWaveformPulseRunnable::findIntersection(
    vector<double> const &tMinMax,
    glm::dvec3 const &o,
    glm::dvec3 const &v
#ifdef DATA_ANALYTICS
   ,std::vector<double> &subraySimRecord
#endif
) const{
    // Consider scene AABB intersection check was done before at operator()
    // Thus, proceed to raycasting directly
    return shared_ptr<RaySceneIntersection>(raycaster->search(
        o, v, tMinMax[0], tMinMax[1], false
#ifdef DATA_ANALYTICS
       ,subraySimRecord,
        true  // Subray flag
#endif
    ));
}
