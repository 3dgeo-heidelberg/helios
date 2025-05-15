#include "maths/Directions.h"
#include <glm/gtx/norm.hpp>

#include "MathConstants.h"
#include "Platform.h"
#include "PrintUtils.h"
#include "Vectorial.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<Platform>
Platform::clone()
{
  std::shared_ptr<Platform> p = std::make_shared<Platform>();
  _clone(p);
  return p;
}
void
Platform::_clone(std::shared_ptr<Platform> p)
{
  p->cfg_device_relativeMountPosition =
    glm::dvec3(this->cfg_device_relativeMountPosition);
  p->cfg_device_relativeMountAttitude =
    Rotation(this->cfg_device_relativeMountAttitude);
  p->lastCheckZ = this->lastCheckZ;
  p->lastGroundCheck = glm::dvec3(this->lastGroundCheck);
  p->scene = scene;

  p->positionXNoiseSource = this->positionXNoiseSource;
  p->positionYNoiseSource = this->positionYNoiseSource;
  p->positionZNoiseSource = this->positionXNoiseSource;
  p->attitudeXNoiseSource = this->attitudeXNoiseSource;
  p->attitudeYNoiseSource = this->attitudeYNoiseSource;
  p->attitudeZNoiseSource = this->attitudeZNoiseSource;

  p->dmax = this->dmax;
  p->prevWrittenPos = glm::dvec3(this->prevWrittenPos);
  p->cfg_settings_movePerSec_m = this->cfg_settings_movePerSec_m;
  p->targetWaypoint = glm::dvec3(this->targetWaypoint);
  p->onGround = this->onGround;
  p->stopAndTurn = this->stopAndTurn;
  p->smoothTurn = this->smoothTurn;
  p->position = glm::dvec3(this->position);
  p->attitude = Rotation(this->attitude);
  p->mSetOrientationOnLegInit = this->mSetOrientationOnLegInit;
  p->cached_absoluteMountPosition =
    glm::dvec3(this->cached_absoluteMountPosition);
  p->cached_absoluteMountAttitude =
    Rotation(this->cached_absoluteMountAttitude);
  p->cached_dir_current = glm::dvec3(this->cached_dir_current);
  p->cached_dir_current_xy = glm::dvec3(this->cached_dir_current_xy);
  p->cached_vectorToTarget = glm::dvec3(this->cached_vectorToTarget);
  p->cached_vectorToTarget_xy = glm::dvec3(this->cached_vectorToTarget_xy);
  p->cached_distanceToTarget_xy = this->cached_distanceToTarget_xy;
  p->cached_originToTargetDir_xy =
    glm::dvec3(this->cached_originToTargetDir_xy);
  p->cached_targetToNextDir_xy = glm::dvec3(this->cached_targetToNextDir_xy);
  p->cached_endTargetAngle_xy = this->cached_endTargetAngle_xy;
  p->cached_currentAngle_xy = this->cached_currentAngle_xy;
  p->cached_originToTargetAngle_xy = this->cached_originToTargetAngle_xy;
  p->cached_targetToNextAngle_xy = this->cached_targetToNextAngle_xy;
}

// ***  M E T H O D S  *** //
// *********************** //
void
Platform::applySettings(std::shared_ptr<PlatformSettings> settings, bool manual)
{
  cfg_settings_movePerSec_m = settings->movePerSec_m;
  onGround = settings->onGround;

  // Set platform position:
  setPosition(settings->getPosition());
}
shared_ptr<PlatformSettings>
Platform::retrieveCurrentSettings()
{
  shared_ptr<PlatformSettings> settings = make_shared<PlatformSettings>();
  // Settings from Platform
  settings->movePerSec_m = cfg_settings_movePerSec_m;
  settings->onGround = onGround;
  settings->setPosition(getPosition());
  // Return settings
  return settings;
}

void
Platform::setAttitude(Rotation attitude)
{
  this->attitude = attitude;
  this->cached_absoluteMountAttitude = this->getDirectionalAttitude().applyTo(
    this->cfg_device_relativeMountAttitude);
  updateDynamicCache();
}

void
Platform::setOrigin(glm::dvec3 const origin)
{
  this->originWaypoint = origin;
  updateStaticCache();
}
void
Platform::setDestination(glm::dvec3 const dest)
{
  this->targetWaypoint = dest;
  updateStaticCache();
}
void
Platform::setAfterDestination(glm::dvec3 const next)
{
  this->nextWaypoint = next;
  updateStaticCache();
}
void
Platform::setPosition(glm::dvec3 const pos)
{
  position = pos;
  cached_absoluteMountPosition = position + cfg_device_relativeMountPosition;
  updateDynamicCache();
}
void
Platform::updateStaticCache()
{
  cached_originToTargetDir_xy =
    glm::normalize(glm::dvec3(targetWaypoint.x - originWaypoint.x,
                              targetWaypoint.y - originWaypoint.y,
                              0));
  cached_targetToNextDir_xy = glm::normalize(glm::dvec3(
    nextWaypoint.x - targetWaypoint.x, nextWaypoint.y - targetWaypoint.y, 0));

  cached_endTargetAngle_xy =
    std::acos(cached_originToTargetDir_xy.x * cached_targetToNextDir_xy.x +
              cached_originToTargetDir_xy.y * cached_targetToNextDir_xy.y);
  if (std::isnan(cached_endTargetAngle_xy))
    cached_endTargetAngle_xy = 0.0;

  cached_originToTargetAngle_xy =
    Vectorial::directionToAngleXY(cached_originToTargetDir_xy, true);

  cached_targetToNextAngle_xy =
    Vectorial::directionToAngleXY(cached_targetToNextDir_xy, true);

  updateDynamicCache();
}

void
Platform::updateDynamicCache()
{
  // Distance/Position vectors
  cached_vectorToTarget = targetWaypoint - position;
  cached_vectorToTarget_xy =
    glm::dvec3(cached_vectorToTarget.x, cached_vectorToTarget.y, 0);

  // Distances
  cached_distanceToTarget_xy = glm::l2Norm(cached_vectorToTarget_xy);

  // Direction vectors
  cached_dir_current = getCurrentDirection();
  cached_dir_current_xy =
    glm::normalize(glm::dvec3(cached_dir_current.x, cached_dir_current.y, 0));
  cached_currentAngle_xy =
    std::acos(cached_dir_current_xy.x * cached_targetToNextDir_xy.x +
              cached_dir_current_xy.y * cached_targetToNextDir_xy.y);
  if (std::isnan(cached_currentAngle_xy))
    cached_currentAngle_xy = 0.0;
}

bool
Platform::waypointReached()
{
  // Stationary platforms are always at their destination:
  return true;
}
