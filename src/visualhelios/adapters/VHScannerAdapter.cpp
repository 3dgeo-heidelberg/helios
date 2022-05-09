#ifdef PCL_BINDING

#include <visualhelios/adapters/VHScannerAdapter.h>
#include <sim/comps/Leg.h>
#include <platform/HelicopterPlatform.h>

#include <glm/gtx/norm.hpp>

using visualhelios::VHScannerAdapter;

// ***  SIMULATION  *** //
// ******************** //
void VHScannerAdapter::start(){
    scanner.platform->prepareSimulation(getPulseFreq_Hz());
    startLeg(0, true);
}
void VHScannerAdapter::nextStep(){
    // Check whether current leg has been completed or not
    if(
        scanner.scannerHead->rotateCompleted() &&
        scanner.platform->waypointReached()
    ){
        std::cout << "Leg "  << currentLegIndex << " completed!" << std::endl;
        // Start next leg, if any
        if(currentLegIndex < survey.legs.size() - 1){
            std::cout   << "Starting leg " << (currentLegIndex+1) << " ..."
                        << std::endl;
            startLeg(currentLegIndex + 1, false);
        }
        return;
    }

    // If current leg is under progress yet, do necessary computations
    scanner.platform->doSimStep(getPulseFreq_Hz());

    // Update head attitude (we do this even when the scanner is inactive):
    scanner.scannerHead->doSimStep(getPulseFreq_Hz());

    // If the scanner is inactive, stop here:
    if (!scanner.isActive()) return;

    // Update beam deflector attitude:
    scanner.beamDeflector->doSimStep();

    // Calculate absolute beam originWaypoint:
    origin =    scanner.platform->getAbsoluteMountPosition() +
                    scanner.cfg_device_headRelativeEmitterPosition;

    // Calculate absolute beam attitude:
    dir = scanner.calcAbsoluteBeamAttitude().applyTo(Directions::forward);
}

void VHScannerAdapter::startLeg(
    unsigned int const legIndex, bool const manual
){
    // Obtain leg to be started
    currentLegIndex = legIndex;
    shared_ptr<Leg> leg = survey.legs.at(currentLegIndex);

    // Apply scanner settings
    if(leg->mScannerSettings != nullptr){
        scanner.applySettings(leg->mScannerSettings);
    }
    scanner.lastTrajectoryTime = 0L;

    // Apply platform settings
    Platform &platform = *(scanner.platform);
    if(leg->mPlatformSettings != nullptr){
        platform.applySettings(leg->mPlatformSettings, manual);

        unsigned int nextLegIndex = legIndex + 1;
        if(nextLegIndex < survey.legs.size()){
            // Set destination to position of next leg
            Leg &nextLeg = *(survey.legs.at(nextLegIndex));
            if(
                leg->mTrajectorySettings != nullptr &&
                leg->mTrajectorySettings->teleportToStart
            ){
                platform.setPosition(nextLeg.mPlatformSettings->getPosition());
                platform.setOrigin(nextLeg.mPlatformSettings->getPosition());
                platform.setDestination(
                    nextLeg.mPlatformSettings->getPosition()
                );
            }
            else{
                platform.setOrigin(leg->mPlatformSettings->getPosition());
                if(
                    nextLeg.mTrajectorySettings != nullptr &&
                    nextLeg.mTrajectorySettings->teleportToStart
                ){
                    // If next leg teleports to start, current leg is stop leg
                    // Thus, set stop origin and destination to the same point
                    platform.setDestination(
                        leg->mPlatformSettings->getPosition()
                    );
                }
                else{
                    platform.setDestination(
                        nextLeg.mPlatformSettings->getPosition()
                    );
                }
            }
            if(nextLegIndex + 1 < survey.legs.size()){
                platform.setAfterDestination(
                    survey.legs.at(nextLegIndex + 1)
                        ->mPlatformSettings->getPosition()
                );
            }
            else{
                platform.setAfterDestination(
                    nextLeg.mPlatformSettings->getPosition()
                );
            }
        }

        if(platform.canStopAndTurn() && leg->mPlatformSettings->stopAndTurn)
            stopAndTurn(legIndex, leg);
        else if(manual) platform.initLegManual();
        else platform.initLeg();

        // Restart deflector if previous leg was not active
        if(currentLegIndex > 0){
            shared_ptr<Leg> previousLeg = survey.legs.at(currentLegIndex-1);
            if(
                previousLeg != nullptr &&
                !previousLeg->mScannerSettings->active &&
                leg->mScannerSettings->active
            ){
                scanner.beamDeflector->restartDeflector();
            }
        }
    }
}

void VHScannerAdapter::stopAndTurn(unsigned int legIndex, shared_ptr<Leg> leg){
    // Check that the platform is a HelicopterPlatform, otherwise return
    HelicopterPlatform *hp = dynamic_cast<HelicopterPlatform*>(
        scanner.platform.get()
    );
    if(hp == nullptr) return;

    if(legIndex == survey.legs.size()-1) return;
    glm::dvec3 currentPos = leg->mPlatformSettings->getPosition();
    glm::dvec3 nextPos =
        survey.legs[legIndex+1]->mPlatformSettings->getPosition();
    glm::dvec3 xyDir = nextPos-currentPos;
    xyDir.z = 0;
    xyDir = glm::normalize(xyDir);
    glm::dvec3 xyRef = glm::dvec3(0,1,0);
    double sign = (xyDir.x >= 0) ? 1 : -1;
    // sign*acos(-dotProduct/magnitudeProduct)
    double angle = sign*std::acos(
        -glm::dot(xyRef,xyDir) / glm::l2Norm(xyRef) / glm::l2Norm(xyDir)
    );

    scanner.platform->setHeadingRad(angle);
    if(hp!=nullptr){
        std::shared_ptr<PlatformSettings> lps = leg->mPlatformSettings;
        if(lps->yawAtDepartureSpecified) hp->yaw = lps->yawAtDeparture;
        hp->pitch = 0.0;
        hp->speed_xy.x = 0.0001;
        hp->speed_xy.y = 0.0001;
    }
}

#endif