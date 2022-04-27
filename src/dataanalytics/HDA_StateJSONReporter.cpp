#include <dataanalytics/HDA_StateJSONReporter.h>
#include <sim/core/SurveyPlayback.h>
#include <main/helios_version.h>
#include <filems/facade/FMSFacade.h>

#include <sstream>

using namespace helios::analytics;
using namespace helios::filems;

// ***  MAIN REPORT METHODS  *** //
// ***************************** //
void HDA_StateJSONReporter::report(){
    // Write start of report
    std::stringstream ss;
    ss  << "{\n"
        << openEntry("helios", 1, EntryType::OBJECT)
        << craftEntry("version", getHeliosVersion(), 2, true)
        << openEntry("state", 2, EntryType::OBJECT)
    ;
    writer.write(ss.str());
    ss.str("");

    // Write report contents
    reportSimulation();
    reportSurvey();
    reportFilems();
    reportPlatform(); // TODO Restore
    //reportScanner(); // TODO Restore
    //reportScene(); // TODO Restore
    //reportLegs(); // TODO Restore

    // Write end of report
    ss  << closeEntry(2, true, EntryType::OBJECT)  // Close state entry
        << closeEntry(1, true, EntryType::OBJECT)  // Close helios entry
        << "}"
        << std::endl;
    writer.write(ss.str());
    writer.finish();
}

// ***  SECONDARY REPORT METHODS  *** //
// ********************************** //
void HDA_StateJSONReporter::reportSimulation(){
    Simulation *sim = (Simulation *) sp;
    std::stringstream ss;
    ss  << openEntry("simulation", 3, EntryType::OBJECT)
        << craftEntry(
            "parallelizationStrategy",
            sim->parallelizationStrategy,
            4
        )
        << craftEntry("simSpeedFactor", sim->getSimSpeedFactor() , 4)
        << craftEntry("simFrequency", sim->getSimFrequency(), 4)
        << craftEntry("callbackFrequency", sim->callbackFrequency, 4)
        << craftEntry("stopped", sim->isStopped(), 4)
        << craftEntry("paused", sim->isPaused(), 4)
        << craftEntry("timeStart_ms", sim->timeStart_ms, 4)
        << craftEntry("currentGpsTime_ms", sim->currentGpsTime_ms, 4)
        << craftEntry("fixedGpsTimeStart", sim->fixedGpsTimeStart, 4, true)
        << craftEntry("currentLegIndex", sim->mCurrentLegIndex, 4)
        << craftEntry("exportToFile", sim->exportToFile, 4)
        << craftEntry("exitAtEnd", sim->exitAtEnd, 4)
        << craftEntry("finished", sim->finished, 4)
        << craftEntry("legStarted", sp->mLegStarted, 4)
        << craftEntry("numEffectiveLegs", sp->getNumEffectiveLegs(), 4)
        << craftEntry("elapsedLength", sp->getElapsedLength(), 4)
        << craftEntry("progress", sp->getProgress(), 4)
        << craftEntry("legProgress", sp->getLegProgress(), 4)
        << craftEntry("legStartTime_ns", sp->getLegStartTime(), 4)
        << craftEntry("elapsedTime_ns", sp->getElapsedTime(), 4)
        << craftEntry("remainingTime_ms", sp->getRemainingTime(), 4)
        << craftEntry("legElapsedTime_ms", sp->getLegElapsedTime(), 4)
        << craftEntry(
            "legRemainingTime_ms", sp->getLegRemainingTime(), 4, false, true
        )
        << closeEntry(3, false, EntryType::OBJECT) // Close simulation
    ;
    writer.write(ss.str());
}

void HDA_StateJSONReporter::reportSurvey(){
    Survey *sv = sp->mSurvey.get();
    std::stringstream ss;
    ss  << openEntry("survey", 3, EntryType::OBJECT)
        << craftEntry("name", sv->name, 4, true)
        << craftEntry("numRuns", sv->numRuns, 4)
        << craftEntry("simSpeedFactor", sv->simSpeedFactor, 4)
        << craftEntry("length", sv->getLength(), 4, false, true)
        << closeEntry(3, false, EntryType::OBJECT) // Close survey
    ;
    writer.write(ss.str());
}

void HDA_StateJSONReporter::reportFilems(){
    FMSFacade *fms = sp->fms.get();
    FMSWriteFacade &wf = fms->write;
    VectorialMeasurementWriter *mw = wf.getMeasurementWriter().get();
    TrajectoryWriter *tw = wf.getTrajectoryWriter().get();
    FullWaveformWriter *fw = wf.getFullWaveformWriter().get();
    std::stringstream ss;
    ss  << openEntry("filems", 3, EntryType::OBJECT)
        << craftEntry("writeRootDir", wf.getRootDir(), 4, true)
        << craftEntry(
            "measurementWriterOutputPath", mw->getOutputPath(), 4, true
            )
        << craftEntry("measurementWriterShift", mw->getShift(), 4)
        << craftEntry("measurementWriterIsLasOutput", mw->isLasOutput(), 4)
        << craftEntry("measurementWriterIsLas10", mw->isLas10(), 4)
        << craftEntry("measurementWriterIsZipOutput", mw->isZipOutput(), 4)
        << craftEntry("measurementWriterLasScale", mw->getLasScale(), 4)
        << craftEntry(
            "trajectoryWriterOutputPath", tw->getOutputPath(), 4, true
            )
        << craftEntry("trajectoryWriterIsLasOutput", tw->isLasOutput(), 4)
        << craftEntry("trajectoryWriterIsLas10", tw->isLas10(), 4)
        << craftEntry("trajectoryWriterIsZipOutput", tw->isZipOutput(), 4)
        << craftEntry("trajectoryWriterLasScale", tw->getLasScale(), 4)
    ;
    if(fw != nullptr && fw->hasWriter()){
        ss  << craftEntry("fwfWriterOutputPath", fw->getOutputPath(), 4, true)
            << craftEntry("fwfWriterIsLasOutput", fw->isLasOutput(), 4)
            << craftEntry("fwfWriterIsLas10", fw->isLas10(), 4)
            << craftEntry("fwfWriterIsZipOutput", fw->isZipOutput(), 4)
            << craftEntry(
                "fwfWriterLasScale", fw->getLasScale(), 4, false, true
            );
    }
    else{
        ss  << craftEntry("fwfWriterOutputPath", "", 4, true)
            << craftEntry("fwfWriterIsLasOutput", false, 4)
            << craftEntry("fwfWriterIsLas10", false, 4)
            << craftEntry("fwfWriterIsZipOutput", false, 4)
            << craftEntry("fwfWriterLasScale", 0.0, 4, false, true);
    }
    ss  << closeEntry(3, false, EntryType::OBJECT); // Close filems
    writer.write(ss.str());
}

void HDA_StateJSONReporter::reportPlatform(){
    Platform *pf = sp->mSurvey->scanner->platform.get();
    double roll, pitch, yaw;
    pf->getRollPitchYaw(roll, pitch, yaw);
    std::stringstream ss;
    ss  << openEntry("platform", 3, EntryType::OBJECT)
        << craftEntry(
            "relativeMountPosition", pf->cfg_device_relativeMountPosition, 4
            )
        << craftEntry(
            "relativeMountAttitude", pf->cfg_device_relativeMountAttitude, 4
            )
        << craftEntry("lastCheckZ", pf->lastCheckZ, 4)
        << craftEntry("lastGroundCheck", pf->lastGroundCheck, 4)
        << craftEntry("dmax", pf->dmax, 4)
        << craftEntry("prevWrittenPos", pf->prevWrittenPos, 4)
        << craftEntry("movePerSec_m", pf->cfg_settings_movePerSec_m, 4)
        << craftEntry("originWaypoint", pf->originWaypoint, 4)
        << craftEntry("targetWaypoint", pf->targetWaypoint, 4)
        << craftEntry("nextWaypoint", pf->nextWaypoint, 4)
        << craftEntry("onGround", pf->onGround, 4)
        << craftEntry("stopAndTurn", pf->stopAndTurn, 4)
        << craftEntry("smoothTurn", pf->smoothTurn, 4)
        << craftEntry("slowdownEnabled", pf->slowdownEnabled, 4)
        << craftEntry("position", pf->position, 4)
        << craftEntry("attitude", pf->attitude, 4)
        << craftEntry(
            "setOrientationOnLegInit", pf->mSetOrientationOnLegInit, 4
            )
        << craftEntry("writeNextTrajectory", pf->writeNextTrajectory, 4)
        << craftEntry(
            "cached_absoluteMountPosition",
            pf->cached_absoluteMountPosition,
            4
            )
        << craftEntry(
            "cached_absoluteMountAttitude",
            pf->cached_absoluteMountAttitude,
            4
            )
        << craftEntry("cached_dir_current", pf->cached_dir_current, 4)
        << craftEntry("cached_dir_current_xy", pf->cached_dir_current_xy, 4)
        << craftEntry("cached_vectorToTarget", pf->cached_vectorToTarget, 4)
        << craftEntry(
            "cached_vectorToTarget_xy", pf->cached_vectorToTarget_xy, 4
            )
        << craftEntry(
            "cached_distanceToTarget_xy", pf->cached_distanceToTarget_xy, 4
            )
        << craftEntry(
            "cached_originToTargetDir_xy", pf->cached_originToTargetDir_xy, 4
            )
        << craftEntry(
            "cached_targetToNextDir_xy", pf->cached_targetToNextDir_xy, 4
            )
        << craftEntry(
            "cached_endTargetAngle_xy", pf->cached_endTargetAngle_xy, 4
            )
        << craftEntry(
            "cached_currentAngle_xy", pf->cached_currentAngle_xy, 4
            )
        << craftEntry(
            "cached_originToTargetAngle_xy",
            pf->cached_originToTargetAngle_xy,
            4
            )
        << craftEntry(
            "cached_originToTargetAngle_xy",
            pf->cached_targetToNextAngle_xy,
            4
            )
        << craftEntry("roll", roll, 4)
        << craftEntry("pitch", pitch, 4)
        << craftEntry("yaw", yaw, 4, false, true)
        << closeEntry(3, false, EntryType::OBJECT); // Close platform
    ;
    writer.write(ss.str());
}

// ***  UTIL METHODS  *** //
// ********************** //
template <typename ValType>
std::string HDA_StateJSONReporter::craftEntry(
    std::string const &key,
    ValType const &val,
    int const depth,
    bool const asString,
    bool const last
){
    std::stringstream ss;
    ss  << openEntry(key, depth, EntryType::VALUE);
    if(asString) ss << "\"";
    ss << val;
    if(asString) ss << "\"";
    if(!last) ss << ",";
    ss << "\n";
    return ss.str();
}

std::string HDA_StateJSONReporter::craftEntry(
    std::string const &key,
    glm::dvec3 const &u,
    int const depth,
    bool const asString,
    bool const last
){
    std::stringstream ss;
    ss << "[" << u.x << ", " << u.y << ", " << u.z << "]";
    return craftEntry(key, ss.str(), depth, asString, last);
}

std::string HDA_StateJSONReporter::craftEntry(
    std::string const &key,
    Rotation const &r,
    int const depth,
    bool const asString,
    bool const last
){
    std::stringstream ss;
    ss  << "[" << r.getQ0() << ", " << r.getQ1() << ", " << r.getQ2()
        << ", " << r.getQ3() << "]";
    return craftEntry(key, ss.str(), depth, asString, last);
}

std::string HDA_StateJSONReporter::openEntry(
    std::string const &key,
    int const depth,
    EntryType const entryType
){
    std::stringstream ss;
    for(int i = 0 ; i < depth ; ++i) ss << "\t";
    ss << "\"" << key << "\": ";
    if(entryType==EntryType::OBJECT){
        ss << "{\n";
    }
    else if(entryType==EntryType::ARRAY){
        ss << "[\n";
    }
    return ss.str();
}
std::string HDA_StateJSONReporter::closeEntry(
    int const depth,
    bool const last,
    EntryType const entryType
){
    if(entryType==EntryType::VALUE) return "";
    std::stringstream ss;
    for(int i = 0 ; i < depth ; ++i) ss << "\t";
    if(entryType==EntryType::OBJECT){
        if(last) ss << "}\n";
        else ss << "},\n";
    }
    else if(entryType==EntryType::ARRAY){
        if(last) ss << "]\n";
        else ss << "],\n";
    }
    return ss.str();
}
