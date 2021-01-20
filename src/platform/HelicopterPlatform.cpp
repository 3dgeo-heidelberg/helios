#include "HelicopterPlatform.h"

#include "maths/Directions.h"

#include <glm/gtx/norm.hpp>
#include <logging.hpp>
#include <maths/MathConstants.h>
#include "PrintUtils.h"
#include "Vectorial.h"

// ***  CONSTRUCTION / DESTRUCTION *** //
// *********************************** //
std::shared_ptr<Platform> HelicopterPlatform::clone(){
    std::shared_ptr<Platform> hp = std::make_shared<HelicopterPlatform>();
    _clone(hp);
    return hp;
};
void HelicopterPlatform::_clone(std::shared_ptr<Platform> p){
    SimplePhysicsPlatform::_clone(p);
    HelicopterPlatform *hp = (HelicopterPlatform *) p.get();
    hp->cfg_slowdown_dist_xy = this->cfg_slowdown_dist_xy;
    hp->cfg_slowdown_magnitude = this->cfg_slowdown_magnitude;
    hp->cfg_speedup_magnitude = this->cfg_speedup_magnitude;
    hp->ef_xy_max = this->ef_xy_max;
    hp->yaw = this->yaw;
    hp->roll = this->roll;
    hp->pitch = this->pitch;
    hp->lastSign = this->lastSign;
    hp->cfg_pitch_base = this->cfg_pitch_base;
    hp->cfg_yaw_speed = this->cfg_yaw_speed;
    hp->cfg_roll_speed = this->cfg_roll_speed;
    hp->cfg_pitch_speed = this->cfg_pitch_speed;
    hp->cfg_max_roll_offset = this->cfg_max_roll_offset;
    hp->cfg_max_pitch_offset = this->cfg_max_pitch_offset;
    hp->cfg_max_pitch = this->cfg_max_pitch;
    hp->cfg_min_pitch = this->cfg_min_pitch;
    hp->cfg_slowdownFactor = this->cfg_slowdownFactor;
    hp->cfg_speedupFactor = this->cfg_speedupFactor;
    hp->cfg_pitchStepMagnitude = this->cfg_pitchStepMagnitude;
    hp->cfg_rollStepMagnitude = this->cfg_rollStepMagnitude;
    hp->cfg_yawStepMagnitude = this->cfg_yawStepMagnitude;
    hp->cache_turnIterations = this->cache_turnIterations;
    hp->cache_turning = this->cache_turning;
    hp->cache_xyDistanceThreshold = this->cache_xyDistanceThreshold;
    hp->cache_speedUpFinished = this->cache_speedUpFinished;
    hp->speed_xy = this->speed_xy;
    hp->lastSpeed_xy = this->lastSpeed_xy;
    hp->r = Rotation(this->r);
    hp->dirAttitudeXY = Rotation(this->dirAttitudeXY);
};

// ***  M E T H O D S  *** //
// *********************** //
void HelicopterPlatform::prepareSimulation(int simFrequency_hz){
    // Compute angular steps magnitude
    cfg_pitchStepMagnitude = cfg_pitch_speed / simFrequency_hz;
    cfg_rollStepMagnitude = cfg_roll_speed / simFrequency_hz;
    cfg_yawStepMagnitude = cfg_yaw_speed / simFrequency_hz;
    cfg_slowdownFactor = 1.0 - cfg_slowdown_magnitude / simFrequency_hz;
    cfg_speedupFactor = 1.0 + cfg_speedup_magnitude / simFrequency_hz;
    Platform::prepareSimulation(simFrequency_hz);
}
void HelicopterPlatform::initLegManual(){
    // Set directional attitude
    try{
        glm::dvec3 targetDirXY = cached_vectorToTarget_xy /
            cached_distanceToTarget_xy;
        dirAttitudeXY = attitude;
        dirAttitudeXY = Rotation(
            cached_dir_current_xy, targetDirXY
        ).applyTo(getDirectionalAttitude());
    }
    catch (std::exception &e) {
        logging::WARN(e.what());
    }

    // Parent manual leg initialization (MovingPlatform)
    MovingPlatform::initLegManual();

    // Initialize angles (roll, pitch, yaw)
    roll = 0.0;
    pitch = cfg_max_pitch;
    yaw = -std::atan2(cached_dir_current_xy.x, cached_dir_current_xy.y);
    if(yaw < 0.0) yaw += PI_2;
}

void HelicopterPlatform::initLeg(){
    cache_aligning = true;
}

bool HelicopterPlatform::waypointReached(){
    if(smoothTurn && cache_turning){
        cache_turnIterations--;
        if(cache_turnIterations == 0){
            cache_turning = false;
            cache_speedUpFinished = false;
            return true;
        }
    }

    if(MovingPlatform::waypointReached()){
        cache_turning = false;
        cache_speedUpFinished = false;
        return true;
    }
    return false;
}

void HelicopterPlatform::updateStaticCache(){
    Platform::updateStaticCache();
    cache_turnIterations = cached_currentAngle_xy / cfg_yawStepMagnitude;
    computeTurnDistanceThreshold();
};

glm::dvec3 HelicopterPlatform::getCurrentDirection(){
    glm::dvec3 xyDir = Platform::getCurrentDirection();
    glm::dvec3 xyzDir = glm::normalize(cached_vectorToTarget);
    double xyNorm = std::sqrt(xyzDir.x*xyzDir.x + xyzDir.y*xyzDir.y);
    return glm::dvec3( // Current direction
        xyDir.x * xyNorm, xyDir.y * xyNorm, xyzDir.z
    );

}

void HelicopterPlatform::computeTurnDistanceThreshold(){
    // SUGGESTION: Add a check to avoid computing last computed TIF ?
    // Prepare computations
    double xMove = 0.0, yMove = 0.0;
    double vxt, vyt;
    double start = Vectorial::directionToAngleXY(cached_dir_current_xy, true);
    // SUGGESTION : Compute speed at turn start to avoid accuracy loss
    double speed = ef_xy_max;
    // SUGGESTION : Compute velocityNorm iteratively
    double velocityNorm = ef_xy_max * mCfg_drag;
    double moveNorm;
    // SUGGESTION: Compute current targetToNextAngle and use instead of cached
    double sign = -Vectorial::shortestRotationSign(
        start,
        cached_targetToNextAngle_xy
    );
    double angle;

    // Computation loop
    for(int t = 0 ; t < cache_turnIterations ; t++){
        // Compute
        angle = start + sign * t * cfg_yawStepMagnitude;
        vxt = sin(angle);
        vyt = cos(angle);
        moveNorm = speed * velocityNorm;
        xMove += vxt * moveNorm;
        yMove += vyt * moveNorm;

        // Prepare next iteration
        speed = computeSlowdownStep(speed);
    }

    // Determine turn start condition
    double xDist = xMove * cached_dir_current_xy.x;
    double yDist = yMove * cached_dir_current_xy.y;
    cache_xyDistanceThreshold = xDist + yDist;
}

// ***  CONTROL STEP  *** //
// ********************** //
void HelicopterPlatform::doControlStep(int simFrequency_hz) {
    // Compute rates/speeds/forces
    double zForceTarget = computeLiftSinkRate();
    computeXYSpeed(simFrequency_hz);
    computeEngineForce(zForceTarget);

    // Rotate
    computeRotationAngles(simFrequency_hz);
    rotate(roll, pitch, yaw);

    // Handle route
    handleRoute(simFrequency_hz);
}

double HelicopterPlatform::computeLiftSinkRate(){
    double zForceTarget = 0;

    // If destination is below:
    if (cached_vectorToTarget.z < 0) {
        zForceTarget = -0.1;

        // Decrease descend speed when approaching waypoint from above:
        if (cached_vectorToTarget.z < 15) {
            zForceTarget = cached_vectorToTarget.z * 0.005;
        }
    }
    else {
        // If destination is above:
        zForceTarget = 0.1;
    }
    return zForceTarget;
}

void HelicopterPlatform::computeXYSpeed(int simFrequency_hz){
    // Prepare XY speed computation
    if(smoothTurn) speed_xy = getCurrentDirection();
    else speed_xy = cached_vectorToTarget;
    speed_xy = glm::normalize(glm::dvec3(speed_xy.x, speed_xy.y, 0.0));
    bool stabilizePitch = true;

    // Slow down on arrival / turning
    if( slowdownEnabled && (
            cache_turning ||
            (!smoothTurn && cached_distanceToTarget_xy < cfg_slowdown_dist_xy)
    )){
        speed_xy = speed_xy * computeSlowdownStep(glm::l2Norm(lastSpeed_xy));
        pitch += cfg_pitchStepMagnitude;
        stabilizePitch = false;
    }
    // Speed up for this leg, as it has not been done yet
    else if(!cache_speedUpFinished){
        //speed_xy = speed_xy * (glm::l2Norm(lastSpeed_xy) + 0.000001);
        double speed = glm::l2Norm(lastSpeed_xy);
        if(speed <= 0.0) speed = 0.0001;
        speed_xy = speed_xy * computeSpeedupStep(speed);
        pitch -= cfg_pitchStepMagnitude;
        stabilizePitch = false;
    }

    // Limit engine power
    if (glm::l2Norm(speed_xy) > ef_xy_max) {
        speed_xy = glm::normalize(speed_xy) * ef_xy_max;
        cache_speedUpFinished = true;
    }

    // Stabilize pitch
    if(stabilizePitch) {
        if (pitch > cfg_pitch_base) pitch -= cfg_pitchStepMagnitude;
        else if (pitch < cfg_pitch_base) pitch += cfg_pitchStepMagnitude;
    }

    lastSpeed_xy = speed_xy;
}

void HelicopterPlatform::computeEngineForce(double zForceTarget){
    double ef_z = -mCfg_g_accel.z + zForceTarget;
    mEngineForce = glm::dvec3(speed_xy.x, speed_xy.y, ef_z);
}

void HelicopterPlatform::computeRotationAngles(int simFrequency_hz){
    // Assure pitch is inside allowed boundaries
    if(pitch > cfg_max_pitch) pitch = cfg_max_pitch;
    else if(pitch < cfg_min_pitch) pitch = cfg_min_pitch;

    // Avoid turning rotations if not turning yet, but stabilize roll
    if(!cache_turning){
        // Stabilize roll if necessary
        if(roll < 0.0) roll += cfg_rollStepMagnitude;
        else if(roll > 0.0) roll -= cfg_rollStepMagnitude;

        // Align if necessary
        if(cache_aligning) computeAlignmentAngles();

        // There is nothing more to do about rotations
        return;
    }

    computeTurningAngles();
}

void HelicopterPlatform::computeAlignmentAngles(){
    // Determine target yaw
    double targetYaw = -Vectorial::directionToAngleXY(
        cached_vectorToTarget_xy
    );

    // Translate current yaw to [0, 2pi)
    if(yaw < 0.0) yaw += std::ceil((-yaw/PI_2)) * PI_2;
    if(yaw > PI_2) yaw -= std::floor(yaw/PI_2) * PI_2;

    // Update yaw
    double sign = -Vectorial::shortestRotationSign(yaw, targetYaw);
    double newYaw = yaw + sign * cfg_yawStepMagnitude;

    // Check yaw update is not too much
    double newSign = -Vectorial::shortestRotationSign(newYaw, targetYaw);
    if(sign != newSign) newYaw = targetYaw;
    yaw = newYaw;

    // Check if more alignment operations will be necessary
    double diff = targetYaw - yaw;
    if(diff > -cfg_alignmentThreshold && diff < cfg_alignmentThreshold){
        cache_aligning = false;
    }
}
void HelicopterPlatform::computeTurningAngles(){
    try {
        // Determine rotation sign
        double sign;
        double currentDirXyAngle = Vectorial::directionToAngleXY(
            cached_dir_current_xy,
            true
        );

        // Sign of rotation from current direction to after target direction
        // For last leg targetToNextDir_xy is nan, so ignore this case
        if (
            !std::isnan(cached_targetToNextDir_xy.x) &&
            !std::isnan(cached_targetToNextDir_xy.y)
            ){
            sign = Vectorial::shortestRotationSign(
                currentDirXyAngle,
                cached_targetToNextAngle_xy
            );
        }
        else { // Sign of rotation from current direction to target direction
            sign = Vectorial::shortestRotationSign(
                currentDirXyAngle,
                cached_originToTargetAngle_xy
            );
        }

        // Perform rotations but not for stopAndTurn mode
        if(!stopAndTurn) {
            // Roll rotation
            if (sign != lastSign) { // Roll : Stabilize
                if (roll < 0.0) roll += cfg_rollStepMagnitude;
                else roll -= cfg_rollStepMagnitude;
            } else { // Roll : Turn
                if (sign > 0.0) roll -= cfg_rollStepMagnitude;
                else roll += cfg_rollStepMagnitude;
            }
            if (roll > cfg_max_roll_offset) roll = cfg_max_roll_offset;
            if (roll < -cfg_max_roll_offset) roll = -cfg_max_roll_offset;

            // Yaw rotation into movement direction
            yaw += sign * cfg_yawStepMagnitude;
        }

        // Cache last sign
        lastSign = sign;
    }
    catch(std::exception &e) {
        std::stringstream ss;
        ss  << "HelicopterPlatform::computeRotationAngles EXCEPTION:\n\t"
            << e .what();
        logging::WARN(ss.str());
    }
}

void HelicopterPlatform::rotate(double roll, double pitch, double yaw){
    try {
        // Platform attitude
        Rotation newAttitude = Rotation(Directions::right, pitch)
            .applyTo(Rotation(Directions::forward, roll));
        r = Rotation(Directions::up, yaw);
        setAttitude(r.applyTo(newAttitude));

        // Directional attitude over XY
        dirAttitudeXY = Rotation(Directions::up, yaw);
    }
    catch(std::exception &e){
        std::stringstream ss;
        ss << "HelicopterPlatform::rotate EXCEPTION:\n\t" << e.what();
        logging::WARN(ss.str());
    }
}

void HelicopterPlatform::handleRoute(int simFrequency_hz) {
    if (
        !cache_turning &&
        cached_distanceToTarget_xy <= cache_xyDistanceThreshold
    ) {
        cache_turning = true;
    }
}
