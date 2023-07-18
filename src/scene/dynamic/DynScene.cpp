#include <scene/dynamic/DynScene.h>
#include <logging.hpp>
#include <SerialIO.h>

using std::stringstream;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
DynScene::DynScene(DynScene &ds) : DynScene(static_cast<StaticScene&>(ds)) {
    for(shared_ptr<DynObject> dynObj : ds.dynObjs){
        dynObjs.push_back(dynObj);
        updated.push_back(true);
    }
    makeStepLoop(ds.stepLoop.getStepInterval());
    stepLoop.setCurrentStep(ds.stepLoop.getCurrentStep());
}

// ***   M E T H O D S   *** //
// ************************* //
void DynScene::prepareSimulation(int const simFrequency_hz){
    // TODO Rethink : To user given parameters ---
    double const dynTimeStep = 0.0002;
    double const partDynTimeStep = 0.001;
    double const kdtDynTimeStep = 0.01;
    // TODO Rethink : Warn about time steps > 1 second
    // TODO Rethink : In asset loading, represent non-given as NaN
    // --- TODO Rethink : To user given parameters
    // Prepare variables
    double const simFreq_hz = (double) simFrequency_hz;
    // Configure the dynamic scene step interval from time
    if(!std::isnan(dynTimeStep)) {
        setStepInterval((int) (simFreq_hz * dynTimeStep) );
    }
    // TODO Rethink : Each scene part must use its own time
    // Configure each dynamic object step interval from time
    for(std::shared_ptr<DynObject> dynObj : dynObjs){
        // Configure the dynamic step interval for each scene part from time
        dynObj->setStepInterval(
            // Compute from previously configured scene step interval
            //(int) (simFreq_hz*partDynTimeStep/getStepInterval())
            // Compute from time ratio
            (int) (partDynTimeStep/dynTimeStep)
        );
        // Configure the dynamic step interval for each observer from time
        // TODO Rethink : Implement below
        std::shared_ptr<DynMovingObject> obs = std::dynamic_pointer_cast<
            DynMovingObject>(dynObj);
        obs->setObserverStepInterval((int) (kdtDynTimeStep/partDynTimeStep));
    }
}

// ***  SIMULATION STEP  *** //
// ************************* //
bool DynScene::doSimStep(){
    if(stepLoop.doStep()) return stepLoop.retrieveOutput();
    return false;
}

bool DynScene::doStep(){
    bool updateFlag = false;
    size_t const n = numDynObjects();
    for(size_t i = 0 ; i < n ; ++i){
        updated[i] = dynObjs[i]->doStep();
        updateFlag |= updated[i];
    }
    return updateFlag;
}

void DynScene::makeStepLoop(int const stepInterval){
    this->stepLoop = NonVoidStepLoop<bool>(
        stepInterval, [&] () -> bool{return doStep();}
    );
}


// ***   READ/WRITE  *** //
// ********************* //
void DynScene::writeObject(std::string path){
    stringstream ss;
    ss << "Writing dynamic scene object to " << path << " ...";
    logging::INFO(ss.str());
    SerialIO::getInstance()->write<DynScene>(path, this);
}
DynScene * DynScene::readObject(std::string path){
    stringstream ss;
    ss << "Reading dynamic scene object from " << path << " ...";
    logging::INFO(ss.str());
    return SerialIO::getInstance()->read<DynScene>(path);
}
