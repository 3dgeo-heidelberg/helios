#include <iostream>

#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <logging.hpp>

#include "maths/Directions.h"

#include "PrintUtils.h"
#include "MovingPlatform.h"
using namespace std;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<Platform> MovingPlatform::clone(){
	std::shared_ptr<Platform> mp = std::make_shared<MovingPlatform>();
	_clone(mp);
	return mp;
}
void MovingPlatform::_clone(std::shared_ptr<Platform> p){
	Platform::_clone(p);
	MovingPlatform *mp = (MovingPlatform *) p.get();
	mp->velocity = glm::dvec3(velocity);
}

// ***  M E T H O D S  *** //
// *********************** //
void MovingPlatform::applySettings(
    std::shared_ptr<PlatformSettings> settings,
    bool manual
){
	cfg_settings_movePerSec_m = settings->movePerSec_m;
    stopAndTurn = settings->stopAndTurn;
    smoothTurn = settings->smoothTurn;
    slowdownEnabled = settings->slowdownEnabled;

	// Set platform position:
	if (manual) {
		setPosition(settings->getPosition());
	}
}

void MovingPlatform::doSimStep(int simFrequency_hz) {
	if (l2Norm(velocity) > 0) {
	    setPosition(position + velocity);
	}
}

void MovingPlatform::initLegManual() {
	// Set Platform Orientation towards destination
	double const eps = 0.025;
	double angle = glm::angle(
	    glm::normalize(cached_vectorToTarget_xy),
        cached_dir_current_xy
    );
	Rotation curr_r = getAttitude();
	Rotation r = curr_r.applyTo(Rotation(Directions::up, angle));
	if(angle > -eps && angle < eps){
	    setAttitude(r);
	    return;
	}
	setAttitude(r);
    angle = glm::angle(
        glm::normalize(cached_vectorToTarget_xy),
        cached_dir_current_xy
    );
    if(angle > eps){ // If direct computation failed
        std::stringstream ss;
        ss  << "It was not possible to determine attitude with a single  "
            << "computation at MovingPlatform::initLegManual\n\t"
            << "angle = " << angle << " but it should below " << eps
            << "\n\tUsing iterative computation instead";
        logging::WARN(ss.str());
        initLegManualIterative();
    }
}

void MovingPlatform::initLegManualIterative(){
    try {
        double angle = glm::angle(
            glm::normalize(cached_vectorToTarget_xy),
            glm::normalize(cached_dir_current_xy)
        );
        double stepSize = 0.025;
        double heading_rad = 0;
        while (angle > stepSize) {
            heading_rad += angle;
            Rotation r = Rotation(Directions::up, heading_rad);
            this->setAttitude(r);
            angle = glm::angle(
                glm::normalize(cached_vectorToTarget_xy),
                glm::normalize(cached_dir_current_xy)
            );
        }
        logging::INFO(
            "Iterative mode was used for manual leg initialization "
            "because default one failed for MovingPlatform"
        );
    }
    catch (std::exception &e) {
        logging::WARN(e.what());
    }
}

bool MovingPlatform::waypointReached() {
	// TODO 5: Make waypoint tolerance configurable
	bool result = glm::l2Norm(cached_vectorToTarget) < 0.000001;
	if (result) logging::INFO("Waypoint reached!");
	return result;
}
