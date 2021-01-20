#include "PolygonMirrorBeamDeflector.h"

#include "maths/Directions.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<AbstractBeamDeflector> PolygonMirrorBeamDeflector::clone(){
    std::shared_ptr<AbstractBeamDeflector> pmbd =
        std::make_shared<PolygonMirrorBeamDeflector>(
            PolygonMirrorBeamDeflector(
                cfg_device_scanFreqMax_Hz,
                cfg_device_scanFreqMin_Hz,
                cfg_device_scanAngleMax_rad,
                cfg_device_scanAngleEffectiveMax_rad
            )
        );
    _clone(pmbd);
    return pmbd;
};
void PolygonMirrorBeamDeflector::_clone(
    std::shared_ptr<AbstractBeamDeflector> abd
){
    AbstractBeamDeflector::_clone(abd);
    PolygonMirrorBeamDeflector *pmbd = (PolygonMirrorBeamDeflector *)abd.get();
    pmbd->cfg_device_scanAngleEffective_rad =
        this->cfg_device_scanAngleEffective_rad;
    pmbd->cfg_device_scanAngleEffectiveMax_rad =
        this->cfg_device_scanAngleEffectiveMax_rad;
};

// ***  M E T H O D S  *** //
// *********************** //
void PolygonMirrorBeamDeflector::doSimStep() {
	// Update beam angle:
	state_currentBeamAngle_rad += cached_angleBetweenPulses_rad;

	if(state_currentBeamAngle_rad >= cfg_setting_verticalAngleMax_rad){
	    state_currentBeamAngle_rad = cfg_setting_verticalAngleMin_rad;
	}


	// Rotate to current position:
	this->cached_emitterRelativeAttitude =
	    Rotation(Directions::right, state_currentBeamAngle_rad);

}

bool PolygonMirrorBeamDeflector::lastPulseLeftDevice() {
	return std::fabs(this->state_currentBeamAngle_rad) <=
	    this->cfg_device_scanAngleEffective_rad;
}