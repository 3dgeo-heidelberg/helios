#include "ScannerHead.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <MathConstants.h>

#include "typedef.h"


// ***  M E T H O D S  *** //
// *********************** //
void ScannerHead::applySettings(std::shared_ptr<ScannerSettings> settings) {
	this->setRotatePerSec_rad(settings->headRotatePerSec_rad);
	this->setCurrentRotateAngle_rad(settings->headRotateStart_rad);
	this->cfg_setting_rotateStop_rad = settings->headRotateStop_rad;
	this->cfg_setting_rotateStart_rad = settings->headRotateStart_rad;
	this->cfg_setting_rotateRange_rad = std::fabs(
	    cfg_setting_rotateStart_rad - cfg_setting_rotateStop_rad
    );
}
void ScannerHead::doSimStep(double pulseFreq_Hz) {
	if (cfg_setting_rotatePerSec_rad != 0) {
		setCurrentRotateAngle_rad(
		    state_currentRotateAngle_rad +
		    cfg_setting_rotatePerSec_rad / pulseFreq_Hz
        );
	}
}

bool ScannerHead::rotateCompleted() {
	bool result = false;

	if (cfg_setting_rotatePerSec_rad < 0) {
		result = state_currentRotateAngle_rad <= cfg_setting_rotateStop_rad;
	}
	else {
		result = state_currentRotateAngle_rad >= cfg_setting_rotateStop_rad;
	}

	return result;
}



// ***  GETTERS and SETTERS  *** //
// ***************************** //
Rotation ScannerHead::getMountRelativeAttitude() {
    return this->cached_mountRelativeAttitude;
}

void ScannerHead::setCurrentRotateAngle_rad(double angle_rad) {
	if (angle_rad == state_currentRotateAngle_rad)
		return;

	state_currentRotateAngle_rad = angle_rad;
	cached_mountRelativeAttitude = Rotation(
	    cfg_device_rotateAxis,
	    fmod(state_currentRotateAngle_rad, PI_2)
    );
}

void ScannerHead::setRotatePerSec_rad(double rotateSpeed_rad) {

	// Limit head rotate speed to device maximum:
	if (std::fabs(rotateSpeed_rad) > cfg_device_rotatePerSecMax_rad) {
		rotateSpeed_rad = sgn(rotateSpeed_rad) * cfg_device_rotatePerSecMax_rad;
	}

	cfg_setting_rotatePerSec_rad = rotateSpeed_rad;
}
