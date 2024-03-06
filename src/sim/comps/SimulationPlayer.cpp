#include <sim/comps/SimulationPlayer.h>
#include <sim/core/Simulation.h>
#include <sim/core/SurveyPlayback.h>
#include <scene/Scene.h>
#include <filems/facade/FMSFacade.h>

#include <memory>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SimulationPlayer::SimulationPlayer(Simulation &sim) :
    sim(sim),
    scene(*sim.getScanner()->platform->scene),
    plays(0),
    platformStart(sim.getScanner()->platform->clone())
{};

// ***  MAIN PUBLIC METHODS  *** //
// ***************************** //
bool SimulationPlayer::hasPendingPlays(){
    return getNumComputedPlays() < getNumTargetPlays();
}

void SimulationPlayer::endPlay(){
    // Update the counts of plays
    ++plays;
    // Get all the scene parts (objects) that will do a swap on repeat
    std::vector<std::shared_ptr<ScenePart>> swapOnRepeatObjects =
        scene.getSwapOnRepeatObjects();
    for(std::shared_ptr<ScenePart> sp : swapOnRepeatObjects){
        std::shared_ptr<SwapOnRepeatHandler> sorh =
            sp->getSwapOnRepeatHandler();
        if(sorh->hasPendingSwaps()) sorh->swap(*sp);
    }
    // Prepare next play, if any
    if(plays < getNumTargetPlays()) {
        // Restart platform
        restartPlatform(*sim.getScanner()->platform);
        // Restart simulation step loop
        sim.getStepLoop().setCurrentStep(0);
        // Restart filems
        restartFileMS(*sim.getScanner()->fms);
        // Restart simulation
        restartSimulation(sim);
    }
}

int SimulationPlayer::getNumTargetPlays(){
    // Get all the scene parts (objects) that will do a swap on repeat
    std::vector<std::shared_ptr<ScenePart>> swapOnRepeatObjects =
        scene.getSwapOnRepeatObjects();
    // The number of target plays is one plus the maximum number of
    // target swaps among the many swap on repeat objects
    int numTargetPlays = 1;
    for(std::shared_ptr<ScenePart> sp : swapOnRepeatObjects) {
        std::shared_ptr<SwapOnRepeatHandler> sorh =
            sp->getSwapOnRepeatHandler();
        if(sorh->getNumTargetSwaps() >= numTargetPlays){
            numTargetPlays = 1 + sorh->getNumTargetSwaps();
        }
    }
    // Return the number of target plays
    return numTargetPlays;
}

int SimulationPlayer::getNumComputedPlays(){
    return plays;
}

// ***  UTIL PROTECTED METHODS  *** //
// ******************************** //
void SimulationPlayer::restartPlatform(Platform &p){
    // TODO Rethink : Strategy 1 ---
    //p = *platformStart;
    // --- TODO Rethink : Strategy 1
    // TODO Rethink : Strategy 2 ---
    p.attitude = platformStart->attitude;
    p.position = platformStart->position;
    p.originWaypoint = platformStart->originWaypoint;
    p.targetWaypoint = platformStart->targetWaypoint;
    // --- TODO Rethink : Strategy 2
}

void SimulationPlayer::restartFileMS(FMSFacade &fms){
    // TODO Rethink : Implement
}

void SimulationPlayer::restartSimulation(Simulation &sim){
    // TODO Rethink : Strategy ---
    SurveyPlayback &sp = static_cast<SurveyPlayback &>(sim);
    sp.progress = 0;
    sp.legProgress = 0;
    sim.finished = false;
    sim.mStopped = false;
    // --- TODO Rethink : Strategy
}
