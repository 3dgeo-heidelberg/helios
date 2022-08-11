#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_SimStepRecorder.h>

#include <util/HeliosException.h>
#include <maths/RotationOrder.h>

#include <boost/filesystem.hpp>
#include <glm/glm.hpp>

#include <sstream>

using namespace helios::analytics;

// ***  MAIN RECORD METHOD  *** //
// **************************** //
void HDA_SimStepRecorder::record(){
    recordPlatform();
    recordScanner();
}

// ***  RECORDER METHODS  *** //
// ************************** //
void HDA_SimStepRecorder::validateOutDir(){
    // Check directory exists
    if(!boost::filesystem::exists(outdir)){
        if(!boost::filesystem::create_directory(outdir)){
            std::stringstream ss;
            ss  << "HDA_SimStepRecorder::validateOutDir thrown an exception "
                << "because the output directory does not exist and cannot be"
                << "created.\noutdir: \"" << outdir << "\"";
            throw HeliosException(ss.str());
        }
    }
}

bool HDA_SimStepRecorder::isAnyBufferOpen(){
    bool anyOpen = false;
    anyOpen |= platformPositionX->isOpen();
    anyOpen |= platformPositionY->isOpen();
    anyOpen |= platformPositionZ->isOpen();
    anyOpen |= platformRoll->isOpen();
    anyOpen |= platformPitch->isOpen();
    anyOpen |= platformYaw->isOpen();
    anyOpen |= scannerPositionX->isOpen();
    anyOpen |= scannerPositionY->isOpen();
    anyOpen |= scannerPositionZ->isOpen();
    anyOpen |= scannerRoll->isOpen();
    anyOpen |= scannerPitch->isOpen();
    anyOpen |= scannerYaw->isOpen();
    return anyOpen;
}

void HDA_SimStepRecorder::openBuffers(){
    // Open platform related buffers
    platformPositionX = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_position_x.csv")
    );
    platformPositionY = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_position_y.csv")
    );
    platformPositionZ = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_position_z.csv")
    );
    platformRoll = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_roll.csv")
    );
    platformPitch = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_pitch.csv")
    );
    platformYaw = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_yaw.csv")
    );

    // Open scanner related buffers
    scannerPositionX = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("scanner_position_x.csv")
    );
    scannerPositionY = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("scanner_position_y.csv")
    );
    scannerPositionZ = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("scanner_position_z.csv")
    );
    scannerRoll = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("scanner_roll.csv")
    );
    scannerPitch = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("scanner_pitch.csv")
    );
    scannerYaw = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("scanner_yaw.csv")
    );
}

void HDA_SimStepRecorder::closeBuffers(){
    // Close platform buffers
    platformPositionX->close();
    platformPositionY->close();
    platformPositionZ->close();
    platformRoll->close();
    platformPitch->close();
    platformYaw->close();

    // Close scanner buffers
    scannerPositionX->close();
    scannerPositionY->close();
    scannerPositionZ->close();
    scannerRoll->close();
    scannerPitch->close();
    scannerYaw->close();
}

// ***  CONCRETE RECORD METHODS  *** //
// ********************************* //
void HDA_SimStepRecorder::recordPlatform(){
    // Obtain platform
    Platform &p = *(sp->getScanner()->platform);
    // Record platform position
    glm::dvec3 const pos = p.getPosition();
    platformPositionX->push(pos.x);
    platformPositionY->push(pos.y);
    platformPositionZ->push(pos.z);
    // Record platform angles
    double roll, pitch, yaw;
    p.getRollPitchYaw(roll, pitch, yaw);
    platformRoll->push(roll);
    platformPitch->push(pitch);
    platformYaw->push(yaw);
}

void HDA_SimStepRecorder::recordScanner(){
    // Obtain scanner
    Scanner &s = *(sp->getScanner());
    // Record scanner position
    glm::dvec3 const pos = s.cfg_device_headRelativeEmitterPosition;
    scannerPositionX->push(pos.x);
    scannerPositionY->push(pos.y);
    scannerPositionZ->push(pos.z);
    // Record scanner angles
    double roll, pitch, yaw;
    s.cfg_device_headRelativeEmitterAttitude.getAngles(
        &RotationOrder::XYZ, roll, pitch, yaw
    );
    scannerRoll->push(roll);
    scannerPitch->push(pitch);
    scannerYaw->push(yaw);
}

std::string HDA_SimStepRecorder::craftOutputPath(std::string const &fname){
    std::stringstream ss;
    ss  << outdir
        << boost::filesystem::path::preferred_separator
        << fname;
    return ss.str();
}

#endif