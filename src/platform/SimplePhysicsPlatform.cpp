#include "SimplePhysicsPlatform.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<Platform> SimplePhysicsPlatform::clone(){
	std::shared_ptr<Platform> spp = std::make_shared<SimplePhysicsPlatform>();
	_clone(spp);
	return spp;
}
void SimplePhysicsPlatform::_clone(std::shared_ptr<Platform> p){
	MovingPlatform::_clone(p);
	SimplePhysicsPlatform *spp = (SimplePhysicsPlatform *) p.get();
	spp->mEngineForce = glm::dvec3(mEngineForce);
	spp->mCfg_g_accel = glm::dvec3(mCfg_g_accel);
	spp->mCfg_drag = mCfg_drag;
}

// ***  M E T H O D S  *** //
// *********************** //
void SimplePhysicsPlatform::doPhysicsStep(int simFrequency_hz) {
	// ############## BEGIN Update vehicle position #################
	glm::dvec3 drag_accel = getVelocity() * (mCfg_drag * simFrequency_hz);
	glm::dvec3 accel = mCfg_g_accel + mEngineForce - drag_accel;
	glm::dvec3 delta_v = accel * (1.0 / simFrequency_hz);

	setVelocity(getVelocity() + delta_v);
	// NOTE: Update of position happens in Platform base class
}

void SimplePhysicsPlatform::doSimStep(int simFrequency_hz) {
	MovingPlatform::doSimStep(simFrequency_hz);
	doControlStep(simFrequency_hz);
	doPhysicsStep(simFrequency_hz);
}

void SimplePhysicsPlatform::doControlStep(int simFrequency_hz) {}

