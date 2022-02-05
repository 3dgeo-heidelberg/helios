#include "OscillatingMirrorBeamDeflector.h"

#include <iostream>
#include <sstream>
using namespace std;

#define _USE_MATH_DEFINES
#include <math.h>
#include <logging.hpp>

#include "maths/Directions.h"
#include "MathConverter.h"


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<AbstractBeamDeflector> OscillatingMirrorBeamDeflector::clone(){
    std::shared_ptr<AbstractBeamDeflector> ombd =
        std::make_shared<OscillatingMirrorBeamDeflector>(
            OscillatingMirrorBeamDeflector(
                cfg_device_scanAngleMax_rad,
                cfg_device_scanFreqMax_Hz,
                cfg_device_scanFreqMin_Hz,
                cfg_device_scanProduct
            )
        );
    _clone(ombd);
    return ombd;
}
void OscillatingMirrorBeamDeflector::_clone(
    std::shared_ptr<AbstractBeamDeflector> abd
){
    AbstractBeamDeflector::_clone(abd);
    OscillatingMirrorBeamDeflector *ombd = (OscillatingMirrorBeamDeflector *)
        abd.get();
    ombd->cfg_setting_scanAngle_rad = cfg_setting_scanAngle_rad;
    ombd->cfg_device_scanProduct = cfg_device_scanProduct;
    ombd->currentScanLinePulse = currentScanLinePulse;
    ombd->cfg_device_turningPulses = cfg_device_turningPulses;
    ombd->cached_pulsesPerScanline = cached_pulsesPerScanline;
    ombd->cached_halfScanlinePulse = cached_halfScanlinePulse;
    ombd->cached_firstAccelerateScanlinePulse =
        cached_firstAccelerateScanlinePulse;
    ombd->cached_firstLinearScanlinePulse = cached_firstLinearScanlinePulse;
    ombd->cached_firstDecelerateScanlinePulse =
        cached_firstDecelerateScanlinePulse;
    ombd->cached_secondAccelerateScanlinePulse =
        cached_secondAccelerateScanlinePulse;
    ombd->cached_secondLinearScanlinePulse = cached_secondLinearScanlinePulse;
    ombd->cached_secondDecelerateScanlinePulse =
        cached_secondDecelerateScanlinePulse;
}

// ***  M E T H O D S  *** //
// *********************** //
void OscillatingMirrorBeamDeflector::applySettings(std::shared_ptr<ScannerSettings> settings) {
	AbstractBeamDeflector::applySettings(settings);
	cached_pulsesPerScanline = (int)
	    (((double)settings->pulseFreq_Hz) / this->cfg_setting_scanFreq_Hz);
	cached_halfScanlinePulse = cached_pulsesPerScanline / 2;
	cached_firstAccelerateScanlinePulse = 0;
	cached_firstLinearScanlinePulse = cfg_device_turningPulses / 2;
	cached_firstDecelerateScanlinePulse = cached_halfScanlinePulse -
	    cfg_device_turningPulses / 2;
	cached_secondAccelerateScanlinePulse = cached_halfScanlinePulse + 1;
	cached_secondLinearScanlinePulse = cached_secondAccelerateScanlinePulse +
	    cfg_device_turningPulses / 2;
	cached_secondDecelerateScanlinePulse = cached_pulsesPerScanline -
	    cfg_device_turningPulses / 2;
    cached_angleBetweenPulses_rad = 2.0 * cfg_setting_scanAngle_rad / (
        (
            (double)(
                cached_firstDecelerateScanlinePulse -
                cached_firstLinearScanlinePulse
            )
        ) + (
            (double)(
                cached_firstLinearScanlinePulse +
                cached_secondAccelerateScanlinePulse -
                cached_firstDecelerateScanlinePulse - 1
            )
        ) / 2.0
    );
}

void OscillatingMirrorBeamDeflector::doSimStep() {
    // Determine pulse number inside current scanline
	if (currentScanLinePulse == cached_pulsesPerScanline) {
		currentScanLinePulse = 0;
	}

	// Update beam angle
	updateBeamAngle();

    // Rotate to current position:
	this->cached_emitterRelativeAttitude = Rotation(Directions::right, state_currentBeamAngle_rad);

    // To next scanline pulse
    currentScanLinePulse++;
}


// ***  UTIL METHODS  *** //
// **********************
void OscillatingMirrorBeamDeflector::updateBeamAngle(){
    // TODO Rethink : Use as many caches as necessary to improve performance
    if(currentScanLinePulse == 0){ // gamma=-theta
        state_currentBeamAngle_rad = -cfg_setting_scanAngle_rad;
    }
    else if(currentScanLinePulse == cached_halfScanlinePulse){ // gamma=+theta
        state_currentBeamAngle_rad = cfg_setting_scanAngle_rad;
    }
    else if( // Positive monotonic accelerate (first accelerate)
        currentScanLinePulse >= cached_firstAccelerateScanlinePulse &&
        currentScanLinePulse < cached_firstLinearScanlinePulse
    ){
        accelerateBeamAngle(
            currentScanLinePulse,
            cached_firstAccelerateScanlinePulse,
            cached_firstLinearScanlinePulse,
            1.0
        );
    }
    else if( // Positive monotonic linear (first linear)
        currentScanLinePulse >= cached_firstLinearScanlinePulse &&
        currentScanLinePulse < cached_firstDecelerateScanlinePulse
    ){
        linearBeamAngle(
            currentScanLinePulse,
            cached_firstLinearScanlinePulse,
            cached_firstDecelerateScanlinePulse,
            1.0
        );
    }
    else if( // Positive monotonic decelerate (first decelerate)
        currentScanLinePulse >= cached_firstDecelerateScanlinePulse &&
        currentScanLinePulse < cached_halfScanlinePulse
    ){
        decelerateBeamAngle(
            currentScanLinePulse,
            cached_firstDecelerateScanlinePulse,
            cached_halfScanlinePulse,
            1.0
        );
    }
    else if( // Negative monotonic accelerate (second accelerate)
        currentScanLinePulse >= cached_secondAccelerateScanlinePulse &&
        currentScanLinePulse < cached_secondLinearScanlinePulse
    ){
        accelerateBeamAngle(
            currentScanLinePulse,
            cached_secondAccelerateScanlinePulse,
            cached_secondLinearScanlinePulse,
            -1.0
        );
    }
    else if( // Negative monotonic linear (second linear)
        currentScanLinePulse >= cached_secondLinearScanlinePulse &&
        currentScanLinePulse < cached_secondDecelerateScanlinePulse
    ){
        linearBeamAngle(
            currentScanLinePulse,
            cached_secondLinearScanlinePulse,
            cached_secondDecelerateScanlinePulse,
            -1.0
        );
    }
    else if( // Negative monotonic decelerate (second decelerate)
        currentScanLinePulse >= cached_secondDecelerateScanlinePulse
    ){
        decelerateBeamAngle(
            currentScanLinePulse,
            cached_secondDecelerateScanlinePulse + 1.9, // Vs. 1.570796 = pi/2,
            cached_pulsesPerScanline,
            -1.0
        );
    }

}
void OscillatingMirrorBeamDeflector::accelerateBeamAngle(
    double const p,
    double const pa,
    double const pb,
    double const sign
){
    state_currentBeamAngle_rad += sign * cached_angleBetweenPulses_rad *
        (p-pa) / (pb-pa)
    ;
}
void OscillatingMirrorBeamDeflector::linearBeamAngle(
    double const p,
    double const pa,
    double const pb,
    double const sign
){
    state_currentBeamAngle_rad += sign * cached_angleBetweenPulses_rad;
}
void OscillatingMirrorBeamDeflector::decelerateBeamAngle(
    double const p,
    double const pa,
    double const pb,
    double const sign
){
    state_currentBeamAngle_rad += sign * cached_angleBetweenPulses_rad *
        (pb-p) / (pb-pa)
    ;
}



// ***  GETTERS and SETTERS  *** //
// ***************************** //
void OscillatingMirrorBeamDeflector::setScanAngle_rad(double scanAngle_rad) {
	double scanAngle_deg = MathConverter::radiansToDegrees(scanAngle_rad);

	// Max. scan angle is limited by scan product:
	if (scanAngle_deg * this->cfg_setting_scanFreq_Hz > this->cfg_device_scanProduct) {
		logging::WARN(
		    "ERROR: Requested scan angle exceeds device limitations "
            "as defined by scan product. "
            "Will set it to maximal possible value."
        );
		scanAngle_deg = ((double) this->cfg_device_scanProduct) / this->cfg_setting_scanFreq_Hz;
	}

	this->cfg_setting_scanAngle_rad = MathConverter::degreesToRadians(scanAngle_deg);
	stringstream ss;
	ss << "Scan angle set to " << scanAngle_deg << " degrees.";
	logging::INFO(ss.str());
}

void OscillatingMirrorBeamDeflector::setScanFreq_Hz(double scanFreq_Hz) {
	// Max. scan frequency is limited by scan product:
	if( MathConverter::radiansToDegrees(this->cfg_setting_scanAngle_rad) *
	    scanFreq_Hz > this->cfg_device_scanProduct
    ){
		logging::WARN(
		    "WARNING: Requested scan frequency exceeds device limitations "
            "as defined by scan product. "
            "Will set it to maximal possible value."
        );
		scanFreq_Hz = ((double) this->cfg_device_scanProduct) /
		    MathConverter::radiansToDegrees(this->cfg_setting_scanAngle_rad);
	}
	this->cfg_setting_scanFreq_Hz = scanFreq_Hz;
	stringstream ss;
	ss << "Scan frequency set to " << this->cfg_setting_scanFreq_Hz << " Hz.";
	logging::INFO(ss.str());
}