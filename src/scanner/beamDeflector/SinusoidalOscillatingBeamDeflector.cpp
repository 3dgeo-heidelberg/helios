#include "SinusoidalOscillatingBeamDeflector.h"

#include <iostream>
#include <sstream>
using namespace std;

#define _USE_MATH_DEFINES
#include <math.h>
#include <logging.hpp>


#include "maths/Directions.h"
#include "MathConverter.h"

using Base = std::shared_ptr<AbstractBeamDeflector>;

// Construction/Cloning
Base SinusoidalOscillatingBeamDeflector::clone(){
    Base ombd =
        std::make_shared<SinusoidalOscillatingBeamDeflector>(
            SinusoidalOscillatingBeamDeflector(
                cfg_device_scanAngleMax_rad,
                cfg_device_scanFreqMax_Hz,
                cfg_device_scanFreqMin_Hz,
                cfg_device_scanProduct
            )
        );
    _clone(ombd);
    return ombd;
}
void SinusoidalOscillatingBeamDeflector::_clone(
    std::shared_ptr<AbstractBeamDeflector> abd
){
    AbstractBeamDeflector::_clone(abd);
    SinusoidalOscillatingBeamDeflector *ombd = (SinusoidalOscillatingBeamDeflector *)
        abd.get();
    ombd->cfg_device_scanProduct = cfg_device_scanProduct;
    ombd->currentScanLinePulse = currentScanLinePulse;
    ombd->scanAngle = scanAngle;
    ombd->cached_pulsesPerScanline = cached_pulsesPerScanline;
}

void SinusoidalOscillatingBeamDeflector::applySettings(std::shared_ptr<ScannerSettings> settings) {
	AbstractBeamDeflector::applySettings(settings);
	cached_angleBetweenPulses_rad = (double)(this->cfg_setting_scanFreq_Hz * this->cfg_setting_scanAngle_rad * 4) / settings->pulseFreq_Hz;
	scanAngle = this->cfg_setting_scanAngle_rad;
	cached_pulsesPerScanline = (int)(((double)settings->pulseFreq_Hz) / this->cfg_setting_scanFreq_Hz);
	deltaT = 1.0 / settings->pulseFreq_Hz;
	cout << "delta T\n" << deltaT;
}

void SinusoidalOscillatingBeamDeflector::doSimStep() {
	
	time += deltaT;

	//time = fmod(time, deltaT);

	// Update beam angle:
	
	state_currentBeamAngle_rad = sin(time * 2.0 * M_PI * cfg_setting_scanFreq_Hz) * scanAngle;

	// Rotate to current position:
	this->cached_emitterRelativeAttitude = Rotation(Directions::right, state_currentBeamAngle_rad);

}

void SinusoidalOscillatingBeamDeflector::setScanAngle_rad(double scanAngle_rad) {
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

void SinusoidalOscillatingBeamDeflector::setScanFreq_Hz(double scanFreq_Hz) {
	// Max. scan frequency is limited by scan product:
	if( MathConverter::radiansToDegrees(this->cfg_setting_scanAngle_rad) *
	    scanFreq_Hz > this->cfg_device_scanProduct
    ){
		logging::WARN(
		    "ERROR: Requested scan frequency exceeds device limitations "
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