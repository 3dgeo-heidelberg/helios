#include "LinearPathPlatform.h"

#include <glm/gtx/norm.hpp>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<Platform> LinearPathPlatform::clone(){
    std::shared_ptr<Platform> lpp = std::make_shared<LinearPathPlatform>();
    _clone(lpp);
    return lpp;
};
void LinearPathPlatform::_clone(std::shared_ptr<Platform> p){
    MovingPlatform::_clone(p);
};


// ***  M E T H O D S  *** //
// *********************** //
void LinearPathPlatform::doSimStep(int simFrequency_hz) {
	MovingPlatform::doSimStep(simFrequency_hz);

	if (glm::l2Norm(getVectorToTarget()) > 0 && cfg_settings_movePerSec_m > 0) {
        // Set Velocity:
        double const speed = cfg_settings_movePerSec_m / simFrequency_hz;
        this->setVelocity(glm::normalize(getVectorToTarget()) * speed);
    }
}

void LinearPathPlatform::setDestination(glm::dvec3 dest) {
	MovingPlatform::setDestination(dest);
	initLegManual();
}

