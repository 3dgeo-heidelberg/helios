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
void SimplePhysicsPlatform::prepareSimulation(int simFrequency_hz) {
  movePerSec_m_stepMagnitude =
    cfg_settings_movePerSec_m / (double)simFrequency_hz;
}

void SimplePhysicsPlatform::checkSpeedLimit()
{
  if (!engineLimitReached) {
    if (userSpeedLimitReached) {
      logging::INFO("User speed (movePerSec_m) reached.");
    } else {
      logging::INFO("Leg is too short to achieve "
                    "the desired (movePerSec_m) speed.");
    }
  } else {
    if (userSpeedLimitReached) {
      logging::INFO("User speed (movePerSec_m) reached.");
    } else {
      logging::INFO("User speed (movePerSec_m) not reached "
                    "due to engine limitations. "
                    "Consider increasing the variable "
                    "engine_max_force in your "
                    "platform settings."); }
  }
  engineLimitReached = false;
  userSpeedLimitReached = false;
}

void SimplePhysicsPlatform::doPhysicsStep(int simFrequency_hz) {
	// ############## BEGIN Update vehicle position #################
	glm::dvec3 drag_accel = getVelocity() * (mCfg_drag * simFrequency_hz);
	glm::dvec3 accel = mCfg_g_accel + mEngineForce - drag_accel;
	glm::dvec3 delta_v = accel * (1.0 / simFrequency_hz);
    glm::dvec3 new_velocity = getVelocity() + delta_v;

    double velocity_magnitude = glm::length(new_velocity);
    double max_velocity_magnitude{};

    if (cfg_settings_movePerSec_m > 0.0) {
      max_velocity_magnitude = movePerSec_m_stepMagnitude;
    }
    else { // If cfg_settings_moverPerSec is not provided by the user
      max_velocity_magnitude = std::numeric_limits<double>::max();
    }

    velocity_magnitude = std::min(velocity_magnitude, max_velocity_magnitude);
    if (!userSpeedLimitReached
        && simFrequency_hz*(max_velocity_magnitude - velocity_magnitude) <= 0.0001) {
      userSpeedLimitReached = true;
    }

    glm::dvec3 velocity = glm::normalize(new_velocity) * velocity_magnitude;
    setVelocity(velocity);

	// NOTE: Update of position happens in Platform base class
}

void SimplePhysicsPlatform::doSimStep(int simFrequency_hz) {
	MovingPlatform::doSimStep(simFrequency_hz);
	doControlStep(simFrequency_hz);
	doPhysicsStep(simFrequency_hz);
}

void SimplePhysicsPlatform::doControlStep(int simFrequency_hz) {}

