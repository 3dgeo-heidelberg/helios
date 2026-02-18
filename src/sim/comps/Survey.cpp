
#include <glm/gtx/norm.hpp>

#include "Survey.h"
#include <AbstractDetector.h>
#include <Simulation.h>
#include <SurveyPlayback.h>
#include <platform/InterpolatedMovingPlatformEgg.h>
#include <scene/StaticScene.h>
#include <scene/dynamic/DynScene.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Survey::Survey(Survey& survey, bool const deepCopy)
{
  // Copy basic attributes
  this->name = survey.name;
  this->numRuns = survey.numRuns;
  this->simSpeedFactor = survey.simSpeedFactor;
  this->length = survey.length;

  // Copy Scanner
  this->scanner = survey.scanner->clone();
  for (size_t i = 0; i < this->scanner->getNumDevices(); ++i) {
    this->scanner->getDetector(i)->scanner = this->scanner;
  }

  // Copy legs
  this->legs = std::vector<std::shared_ptr<Leg>>(0);
  for (size_t i = 0; i < survey.legs.size(); i++) {
    this->legs.push_back(std::make_shared<Leg>(*survey.legs[i]));
  }

  // Make deep copy effective
  if (deepCopy && this->scanner->platform->scene != nullptr) {
    auto scene = this->scanner->platform->scene;
    if (auto dynScene = std::dynamic_pointer_cast<DynScene>(scene)) {
      this->scanner->platform->scene = std::make_shared<DynScene>(*dynScene);
    } else if (auto staticScene =
                 std::dynamic_pointer_cast<StaticScene>(scene)) {
      this->scanner->platform->scene =
        std::make_shared<StaticScene>(*staticScene);
    } else {
      this->scanner->platform->scene = std::make_shared<Scene>(*scene);
    }
  }
}

// ***  M E T H O D S  *** //
// *********************** //
void
Survey::addLeg(int insertIndex, std::shared_ptr<Leg> leg)
{
  if (std::find(legs.begin(), legs.end(), leg) == legs.end()) {
    legs.insert(legs.begin() + insertIndex, leg);
  }
}

void
Survey::removeLeg(int legIndex)
{
  legs.erase(legs.begin() + legIndex);
}

void
Survey::calculateLength()
{
  length = 0;
  for (size_t i = 0; i < legs.size() - 1; i++) {
    legs[i]->setLength(
      glm::distance(legs[i]->mPlatformSettings->getPosition(),
                    legs[i + 1]->mPlatformSettings->getPosition()));
    length += legs[i]->getLength();
  }
}

double
Survey::getLength()
{
  return this->length;
}

void
Survey::hatch(SurveyPlayback& sp)
{
  if (scanner->platform->isEgg()) {
    scanner->platform =
      std::static_pointer_cast<InterpolatedMovingPlatformEgg>(scanner->platform)
        ->smartHatch(sp.getStepLoop());
  }
}
