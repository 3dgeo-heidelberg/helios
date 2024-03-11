#include <sim/comps/SimulationPlayer.h>
#include <sim/core/Simulation.h>
#include <sim/core/SurveyPlayback.h>
#include <scene/Scene.h>
#include <filems/facade/FMSFacade.h>
#include <filems/factory/FMSFacadeFactory.h>
using helios::filems::FMSFacadeFactory;
#include <scanner/Scanner.h>

#include <memory>

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
    // TODO Remove : Debug print ---
    std::cout << "BEFORE:\n";
    for(int i = 0 ; i < scene.primitives.size() ; ++i){
        Primitive * p = scene.primitives[i];
        std::cout << "P[" << i << "] (part " << scene.primitives[i]->part->mId << "):  ";
        for(int j = 0 ; j < p->getNumVertices() ; ++j){
            Vertex & v = p->getVertices()[j];
            std::cout   << "\t(" << v.pos.x << ", " << v.pos.y << ", "
                        << v.pos.z << ") ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    // --- TODO Remove : Debug print
    for(std::shared_ptr<ScenePart> sp : swapOnRepeatObjects){
        std::shared_ptr<SwapOnRepeatHandler> sorh =
            sp->getSwapOnRepeatHandler();
        if(sorh->hasPendingSwaps()){
            // TODO Rethink : Testing full backup for filters ---
            // Backup rotation (because RotateFilter modifies inplace)
            Rotation const rotationBackup = sp->mRotation;
            /*double const scaleBackup = sp->mScale;
            glm::dvec3 const originBackup = sp->mOrigin;*/
            std::cout << "\nsp pre-swap reflectance: " << sp->mPrimitives[0]->material->reflectance << std::endl; // TODO Remove
            // --- TODO Rethink : Testing full backup for filters
            // Do the swap
            sorh->swap(*sp);
            std::cout << "sp rotation angle: " << sp->mRotation.getAngle() << std::endl; // TODO Remove
            std::cout << "sp scale: " << sp->mScale << std::endl; // TODO Remove
            std::cout << "sp origin: " << sp->mOrigin << std::endl; // TODO Remove
            std::cout << "sp material pointer: " << sp->mPrimitives[0]->material << std::endl; // TODO Remove
            std::cout << "sp reflectance: " << sp->mPrimitives[0]->material->reflectance << std::endl; // TODO Remove
            std::cout << "sp pointer: " << sp.get() << "\n" << std::endl; // TODO Remove
            // TODO Rethink : Testing full backup for filters ---
            // Backup rotation (because RotateFilter modifies inplace)
            sp->mRotation = rotationBackup;
            /*sp->mScale = scaleBackup;
            sp->mOrigin = originBackup;*/
            // --- TODO Rethink : Testing full backup for filters
        }
    }
    // Prepare next play, if any
    if(plays < getNumTargetPlays()) {
        // Restart platform
        restartPlatform(*sim.getScanner()->platform);
        // Restart filems
        restartFileMS(*sim.getScanner()->fms);
        // Restart scanner
        restartScanner(*sim.getScanner());
        // Restar scene
        restartScene(*sim.getScanner()->platform->scene);
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
    // TODO Rethink : Implement
    // TODO Rethink : Strategy ---
    sim.mStopped = true;
    sim.getScanner()->getDetector()->shutdown();
    // --- TODO Rethink : Strategy
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
    // TODO Rethink : Strategy ---
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
    fms.write = newFacade->write;
    // TODO Rethink : Replace fms writer with newFacade writer
    // --- TODO Rethink : Strategy
}

void SimulationPlayer::restartScanner(Scanner &sc){
    // TODO Rethink : Implement
    // TODO Rethink : Strategy ---
    // Restart scanner head
    ScannerHead &sh = *sc.getScannerHead();
    sh.setCurrentRotateAngle_rad(sh.getRotateStart());
    // --- TODO Rethink : Strategy

}

void SimulationPlayer::restartScene(Scene &scene){
    // TODO Remove : Debug print ---
    std::cout << "MID:\n";
    for(int i = 0 ; i < scene.primitives.size() ; ++i){
        Primitive * p = scene.primitives[i];
        std::cout << "P[" << i << "] (part " << scene.primitives[i]->part->mId << "):  ";
        for(int j = 0 ; j < p->getNumVertices() ; ++j){
            Vertex & v = p->getVertices()[j];
            std::cout   << "\t(" << v.pos.x << ", " << v.pos.y << ", "
                        << v.pos.z << ") ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    // --- TODO Remove : Debug print
    // TODO Rethink : Implement
    // TODO Rethink : Strategy --
    // Discard scene parts that should be null for the next play, and
    // get primitives from current scene parts (thus, those that belong to
    // non existent scene parts will be discarded)
    std::vector<std::shared_ptr<ScenePart>> newParts;
    std::vector<Primitive *> newPrims;
    glm::dvec3 diff = scene.getBBoxCRS()->getMin(); // TODO Rethink : Move before the loop
    std::cout << "\ndiff = " << diff << "\n" << std::endl; // TODO Remove
    for(std::shared_ptr<ScenePart> sp : scene.parts){
        // Handle scene parts that need to be discarded
        if(sp->sorh != nullptr && sp->sorh->needsDiscardOnReplay()){
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
            // TODO Rethink : What if step=2, this should not be computed more than once
            ScenePart::computeTransformations(sp, sp->sorh->isHolistic());  // TODO Restore
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
    // TODO Remove : Debug print ---
    std::cout << "PRE-AFTER:\n";
    for(int i = 0 ; i < scene.primitives.size() ; ++i){
        Primitive * p = scene.primitives[i];
        std::cout << "P[" << i << "] (part " << scene.primitives[i]->part->mId << "):  ";
        for(int j = 0 ; j < p->getNumVertices() ; ++j){
            Vertex & v = p->getVertices()[j];
            std::cout   << "\t(" << v.pos.x << ", " << v.pos.y << ", "
                        << v.pos.z << ") ";
        }
        std::cout << std::endl;
    }
    std::cout << "\n-------------------------------------\n\n" << std::endl;
    // --- TODO Remove : Debug print
    // Reload scene
    scene.finalizeLoading(false);
    // TODO Remove : Debug print ---
    std::cout << "AFTER:\n";
    for(size_t i = 0 ; i < scene.primitives.size() ; ++i){
        Primitive * p = scene.primitives[i];
        std::cout << "P[" << i << "] (part " << scene.primitives[i]->part->mId << "):  ";
        for(size_t j = 0 ; j < p->getNumVertices() ; ++j){
            Vertex & v = p->getVertices()[j];
            std::cout   << "\t(" << v.pos.x << ", " << v.pos.y << ", "
                        << v.pos.z << ") ";
        }
        std::cout << std::endl;
    }
    std::cout << "\n-------------------------------------\n\n" << std::endl;
    // --- TODO Remove : Debug print
    // --- TODO Rethink : Strategy
}

void SimulationPlayer::restartSimulation(Simulation &sim){
    // TODO Rethink : Implement
    // TODO Rethink : Strategy ---
    // Restart survey playback attributes
    SurveyPlayback &sp = static_cast<SurveyPlayback &>(sim);
    sp.progress = 0;
    sp.legProgress = 0;
    sp.prepareOutput();
    // Restart simulation attributes
    sim.finished = false;
    sim.mStopped = false;
    // Restart simulation step loop
    sim.getStepLoop().setCurrentStep(0);
    // --- TODO Rethink : Strategy
}
