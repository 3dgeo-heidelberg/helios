
#include <glm/gtx/norm.hpp>

#include "Survey.h"
#include <AbstractDetector.h>
#include <SurveyPlayback.h>
#include <Simulation.h>
#include <platform/InterpolatedMovingPlatformEgg.h>


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Survey::Survey(Survey &survey){
    // Copy basic attributes
    this->name = survey.name;
    this->numRuns = survey.numRuns;
    this->simSpeedFactor = survey.simSpeedFactor;
    this->length = survey.length;

    // Copy Scanner
    this->scanner = survey.scanner->clone();
    this->scanner->detector->scanner = this->scanner;

    // Copy legs
    this->legs = std::vector<std::shared_ptr<Leg>>(0);
    for(size_t i = 0 ; i < survey.legs.size() ; i++){
        this->legs.push_back(
            std::make_shared<Leg>(*survey.legs[i])
        );
    }
}

// ***  M E T H O D S  *** //
// *********************** //
void Survey::addLeg(int insertIndex, std::shared_ptr<Leg> leg) {
	if (std::find(legs.begin(), legs.end(), leg) == legs.end()) {
		legs.insert(legs.begin()+insertIndex, leg);
	}
}

void Survey::removeLeg(int legIndex) {
	legs.erase(legs.begin() + legIndex);
}

void Survey::calculateLength() {
    length = 0;
	for (size_t i = 0; i < legs.size() - 1; i++) {
		legs[i]->setLength(
		    glm::distance(
		        legs[i]->mPlatformSettings->getPosition(),
		        legs[i + 1]->mPlatformSettings->getPosition()
		    )
        );
		length += legs[i]->getLength();
	}
}

double Survey::getLength() {
	return this->length;
}

void Survey::hatch(SurveyPlayback &sp){
    if(scanner->platform->isEgg()){
        scanner->platform =
            std::static_pointer_cast<InterpolatedMovingPlatformEgg>(
                scanner->platform
            )->smartHatch(sp.getStepLoop());
    }
}
