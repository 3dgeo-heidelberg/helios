#include "GroundVehiclePlatform.h"

using namespace std;

#include <glm/gtx/norm.hpp>
#include <logging.hpp>

#include "typedef.h"

#include "maths/Directions.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<Platform> GroundVehiclePlatform::clone(){
	std::shared_ptr<Platform> gvp = std::make_shared<GroundVehiclePlatform>();
	_clone(gvp);
	return gvp;
}
void GroundVehiclePlatform::_clone(std::shared_ptr<Platform> p){
	SimplePhysicsPlatform::_clone(p);
	GroundVehiclePlatform *gvp = (GroundVehiclePlatform *) p.get();
	gvp->mEngineForceMax = this->mEngineForceMax;
	gvp->mEngineForceCurrent = this->mEngineForceCurrent;
	gvp->mEngineForceTarget = this->mEngineForceTarget;
	gvp->mComplexTurnThreshold_rad = this->mComplexTurnThreshold_rad;
	gvp->mTurnMode = this->mTurnMode;
	gvp->mTempWaypoint = glm::dvec3(this->mTempWaypoint);
};

// ***  M E T H O D S  *** //
// *********************** //
void GroundVehiclePlatform::doControlStep(int simFrequency_hz) {
	// ################## BEGIN Steering and engine power #################

	double headingChange_rad = 0;

	// Normal forward driving and steering for wide-angle curves:
	if (mTurnMode == 0) {

		if (cached_distanceToTarget_xy > 0) {

			Rotation r1 = Rotation(cached_vectorToTarget_xy, cached_dir_current_xy);
			double angle = r1.getAngle();
			mEngineForceTarget = 0.02;

			if (angle < mComplexTurnThreshold_rad) {
				double sign = (r1.getAxis().z < 0) ? 1 : -1;
				headingChange_rad += 0.1 * sign * glm::l2Norm(this->getVelocity());
			}
			else {
				mTurnMode = 1;
				logging::INFO("Turn mode 1");
				mTempWaypoint = getPosition() + (cached_dir_current * 1.0);
			}
		}
	}

	// Two-step turn for narrow-angle curves, step 1:
	else if (mTurnMode == 1) {

		if (glm::distance(getPosition(), mTempWaypoint) < 0.5) {
			mTurnMode = 2;
			logging::INFO("Turn mode 2");
		}
	}

	// Two-step turn for narrow-angle curves, step 2:
	else if (mTurnMode == 2) {
		Rotation r1 = Rotation(cached_vectorToTarget_xy, cached_dir_current_xy);
		double angle = r1.getAngle();

		if (angle < mComplexTurnThreshold_rad) {
			mTurnMode = 0;
		}
		else {
			double sign = (r1.getAxis().z < 0) ? -1 : 1;
			//headingChange_rad -= 0.25 * sign * this.getVelocity().getNorm();
			headingChange_rad -= 0.25 * sign * glm::l2Norm(this->getVelocity());
			mEngineForceTarget = -0.01;
		}
	}
	// ################## END Steering and engine power #################

	// Set platform attitude:
	this->setAttitude(
	    Rotation(Directions::up, headingChange_rad).applyTo(getAttitude())
    );

	// ########## BEGIN Set engine force #########
	mEngineForceCurrent += sgn(mEngineForceTarget - mEngineForceCurrent)
	    * 0.0001;

	// Limit engine forward/backward force:
	if (mEngineForceCurrent > mEngineForceMax) {
		mEngineForceCurrent = mEngineForceMax;
	}

	if (mEngineForceCurrent < -mEngineForceMax) {
		mEngineForceCurrent = -mEngineForceMax;
	}

	//if (cached_dir_current.getNorm() > 0) {
	if (glm::l2Norm(cached_dir_current) > 0) {
		mEngineForce = glm::normalize(cached_dir_current_xy) * mEngineForceCurrent;
	}
	else {
		mEngineForce = glm::dvec3(0, 0, 0);
	}
	// ########## END Set engine force #########
}

void GroundVehiclePlatform::setDestination(glm::dvec3 dest) {
	Platform::setDestination(dest);
	mTurnMode = 0;
}