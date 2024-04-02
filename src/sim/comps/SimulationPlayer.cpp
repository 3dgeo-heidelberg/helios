#include <sim/comps/SimulationPlayer.h>
#include <sim/core/Simulation.h>
#include <sim/core/SurveyPlayback.h>
#include <scene/Scene.h>
#include <filems/facade/FMSFacade.h>
#include <filems/factory/FMSFacadeFactory.h>
using helios::filems::FMSFacadeFactory;
#include <scanner/Scanner.h>
#include <platform/MovingPlatform.h>
#include <platform/InterpolatedMovingPlatform.h>

#include <memory>
#include <chrono>
using namespace std::chrono;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SimulationPlayer::SimulationPlayer(Simulation &sim) :
    sim(sim),
    scene(*sim.getScanner()->platform->scene),
    plays(0),
    platformStart(sim.getScanner()->platform->clone())
{}

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
    std::stringstream ss;
    for(std::shared_ptr<ScenePart> sp : swapOnRepeatObjects){
        ss.str("");
        ss << "Swapping scene part \"" << sp->mId << "\" ...";
        logging::DEBUG(ss.str());
        std::shared_ptr<SwapOnRepeatHandler> sorh =
            sp->getSwapOnRepeatHandler();
        if(sorh->hasPendingSwaps()){
            ss.str("");
            ss << "Scene part \"" << sp->mId << "\" has pending swaps.";
            logging::DEBUG(ss.str());
            // Backup rotation (because RotateFilter modifies inplace)
            Rotation const rotationBackup = sp->mRotation;
            // Do the swap
            sorh->swap(*sp);
            // Backup rotation (because RotateFilter modifies inplace)
            sp->mRotation = rotationBackup;
        }
        ss.str("");
        ss << "Swapped scene part \"" << sp->mId << "\"!";
        logging::DEBUG(ss.str());
    }
    // Prepare next play, if any
    if(plays < getNumTargetPlays()) {
        logging::DEBUG("Preparing next simulation play ...");
        // Restart platform
        logging::DEBUG("Restarting platform for next simulation play ...");
        restartPlatform(*sim.getScanner()->platform);
        // Restart filems
        logging::DEBUG(
            "Restarting file management system for next simulation play ..."
        );
        restartFileMS(*sim.getScanner()->fms);
        // Restart scanner
        logging::DEBUG("Restarting scanner for next simulation play ...");
        restartScanner(*sim.getScanner());
        // Restar scene
        logging::DEBUG("Restarting scene for next simulation play ...");
        restartScene(*sim.getScanner()->platform->scene);
        // Restart simulation
        logging::DEBUG("Restarting context for next simulation play ...");
        restartSimulation(sim);
        logging::DEBUG("Next simulation play prepared.");
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
            numTargetPlays = 1 + sorh->getNumTargetReplays();
        }
    }
    // Return the number of target plays
    return numTargetPlays;
}

int SimulationPlayer::getNumComputedPlays(){
    return plays;
}

void SimulationPlayer::prepareRepeat(){
    // Handle end of current simulation play to prepare next replay
    sim.mStopped = true;
    sim.getScanner()->getDetector()->shutdown();
}

// ***  UTIL PROTECTED METHODS  *** //
// ******************************** //
void SimulationPlayer::restartPlatform(Platform &p){
    // Restart general platform
    p.attitude = platformStart->attitude;
    p.position = platformStart->position;
    p.originWaypoint = platformStart->originWaypoint;
    p.targetWaypoint = platformStart->targetWaypoint;
    // Restart moving platform
    try {
        MovingPlatform &mp = dynamic_cast<MovingPlatform &>(p);
        std::shared_ptr<MovingPlatform> mpStart =
            std::dynamic_pointer_cast<MovingPlatform>(platformStart);
        mp.setVelocity(mpStart->getVelocity());
        mp.cached_vectorToTarget = mpStart->cached_vectorToTarget;
    }
    catch(std::bad_cast &bcex){}
    // Restart interpolated moving platform
    try{
        InterpolatedMovingPlatform &imp =
            dynamic_cast<InterpolatedMovingPlatform &>(p);
        imp.getTrajectoryFunctionRef().setLastTime(0);
    }
    catch(std::bad_cast &bcex){}
}

void SimulationPlayer::restartFileMS(FMSFacade &fms){
    // Build new facade with updated output directory
    std::shared_ptr<FMSFacade> newFacade = FMSFacadeFactory().buildFacade(
        fms.write.getOutDir(),
        fms.write.getMeasurementWriterLasScale(),
        fms.write.isMeasurementWriterLasOutput(),
        fms.write.isMeasurementWriterLas10(),
        fms.write.isMeasurementWriterZipOutput(),
        fms.write.isSplitByChannel(),
        *static_cast<SurveyPlayback &>(sim).mSurvey,
        false
    );
    // Replace writer of current facade with writer from new facade
    fms.write = newFacade->write;
}

void SimulationPlayer::restartScanner(Scanner &sc){
    // Restart scanner head
    ScannerHead &sh = *sc.getScannerHead();
    sh.setCurrentRotateAngle_rad(sh.getRotateStart());
}

void SimulationPlayer::restartScene(Scene &scene){
    // Discard scene parts that should be null for the next play, and
    // get primitives from current scene parts (thus, those that belong to
    // non existent scene parts will be discarded)
    std::vector<std::shared_ptr<ScenePart>> newParts;
    std::vector<Primitive *> newPrims;
    glm::dvec3 const diff = scene.getBBoxCRS()->getMin();
    for(std::shared_ptr<ScenePart> sp : scene.parts){
        // Handle scene parts that need to be discarded
        if(sp->sorh != nullptr && sp->sorh->needsDiscardOnReplay()){
            for(Primitive * p: sp->sorh->getBaselinePrimitives()) delete p;
            for(Primitive * p: sp->mPrimitives) delete p;
            sp->sorh = nullptr;
            continue;
        }
        // Handle scene parts that must be preserved
        if(sp->sorh == nullptr || (
            sp->sorh != nullptr && !sp->sorh->isOnSwapFirstPlay()
        )){
            // Undo old CRS shift on primitives before scene update and reload
            for(Primitive * p : sp->mPrimitives){
                Vertex *v = p->getVertices();
                for(size_t i = 0 ; i < p->getNumVertices() ; ++i){
                    v[i].pos = v[i].pos + diff;
                }
                p->update();
            }
        }
        // Handle scene parts who are in the first play after a swap
        if(sp->sorh != nullptr && sp->sorh->isOnSwapFirstPlay()) {
            ScenePart::computeTransformations(sp, sp->sorh->isHolistic());
            sp->sorh->setOnSwapFirstPlay(false);
        }
        // Prepare new data for scene
        newParts.push_back(sp);
        newPrims.insert(
            newPrims.cend(),
            sp->mPrimitives.begin(),
            sp->mPrimitives.end()
        );
    }
    scene.parts = newParts;
    scene.primitives = newPrims;
    // Reload scene
    scene.finalizeLoading(false);
}

void SimulationPlayer::restartSimulation(Simulation &sim){
    // Restart survey playback attributes
    logging::DEBUG("Restarting survey playback attributes ...");
    SurveyPlayback &sp = static_cast<SurveyPlayback &>(sim);
    sp.progress = 0;
    sp.legProgress = 0;
    sp.mLegStarted = true;
    sp.legProgress = 0;
    sp.legStartTime_ns = duration_cast<nanoseconds>(
        system_clock::now().time_since_epoch()
    );
    sp.timeStart_ns = sp.legStartTime_ns;
    sp.elapsedLength = 0;
    // Restart simulation attributes
    logging::DEBUG("Restarting simulation attributes ...");
    sim.finished = false;
    sim.mStopped = false;
    sim.mCurrentLegIndex = 0;
    // Restart simulation step loop (i.e., time)
    logging::DEBUG("Restarting simulation step loop ...");
    sim.getStepLoop().setCurrentStep(0);
    // Start first leg
    logging::DEBUG("Restarting first leg ...");
    sp.startLeg(sp.mCurrentLegIndex, true);
}
