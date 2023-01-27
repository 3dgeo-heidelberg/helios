#ifdef DATA_ANALYTICS
#include <dataanalytics/HDA_SimStepRecorder.h>

#include <util/HeliosException.h>
#include <maths/RotationOrder.h>
#include <platform/Platform.h>
#include <scanner/Scanner.h>
#include <scanner/ScannerHead.h>
#include <beamDeflector/AbstractBeamDeflector.h>
#include <maths/MathConstants.h>
#include <util/logger/logging.hpp>
#include <scanner/WarehouseScanningPulseProcess.h>
#include <scanner/BuddingScanningPulseProcess.h>

#include <boost/filesystem.hpp>
#include <glm/glm.hpp>

#include <sstream>

using namespace helios::analytics;

// ***  MAIN RECORD METHOD  *** //
// **************************** //
void HDA_SimStepRecorder::record(){
    recordPlatform();
    recordPlatformMount();
    recordScanner();
    recordScannerHead();
    recordDeflector();
    recordBeam();
}

void HDA_SimStepRecorder::delayedRecord(){
    recordStochastic();
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
    anyOpen |= platformMountX->isOpen();
    anyOpen |= platformMountY->isOpen();
    anyOpen |= platformMountZ->isOpen();
    anyOpen |= platformMountRoll->isOpen();
    anyOpen |= platformMountPitch->isOpen();
    anyOpen |= platformMountYaw->isOpen();
    anyOpen |= scannerPositionX->isOpen();
    anyOpen |= scannerPositionY->isOpen();
    anyOpen |= scannerPositionZ->isOpen();
    anyOpen |= scannerRoll->isOpen();
    anyOpen |= scannerPitch->isOpen();
    anyOpen |= scannerYaw->isOpen();
    anyOpen |= scannerHeadRoll->isOpen();
    anyOpen |= scannerHeadPitch->isOpen();
    anyOpen |= scannerHeadYaw->isOpen();
    anyOpen |= deflectorEmittingRoll->isOpen();
    anyOpen |= deflectorEmittingPitch->isOpen();
    anyOpen |= deflectorEmittingYaw->isOpen();
    anyOpen |= beamOriginX->isOpen();
    anyOpen |= beamOriginY->isOpen();
    anyOpen |= beamOriginZ->isOpen();
    anyOpen |= beamRoll->isOpen();
    anyOpen |= beamPitch->isOpen();
    anyOpen |= beamYaw->isOpen();
    anyOpen |= measErrSeq->isOpen();
    anyOpen |= measErrPar->isOpen();
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

    // Open platform's absolute mount related buffers
    platformMountX = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_mount_x.csv")
    );
    platformMountY = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_mount_y.csv")
    );
    platformMountZ = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_mount_z.csv")
    );
    platformMountRoll = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_mount_roll.csv")
    );
    platformMountPitch = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_mount_pitch.csv")
    );
    platformMountYaw = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("platform_mount_yaw.csv")
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

    // Open scanner's head related buffers
    scannerHeadRoll = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("scanner_head_roll.csv")
    );
    scannerHeadPitch = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("scanner_head_pitch.csv")
    );
    scannerHeadYaw = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("scanner_head_yaw.csv")
    );

    // Open deflector's pulses related buffers
    deflectorEmittingRoll = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("deflector_emitting_roll.csv")
    );
    deflectorEmittingPitch = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("deflector_emitting_pitch.csv")
    );
    deflectorEmittingYaw = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("deflector_emitting_yaw.csv")
    );

    // Open platform related buffers
    beamOriginX = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("beam_origin_x.csv")
    );
    beamOriginY = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("beam_origin_y.csv")
    );
    beamOriginZ = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("beam_origin_z.csv")
    );
    beamRoll = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("beam_roll.csv")
    );
    beamPitch = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("beam_pitch.csv")
    );
    beamYaw = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("beam_yaw.csv")
    );

    // Open stochastic measurements buffers
    measErrSeq = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("measurement_error_sequential.csv")
    );
    measErrPar = std::make_shared<HDA_RecordBuffer<double>>(
        craftOutputPath("measurement_error_parallel.csv")
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

    // Close platform's absolute mount buffers
    platformMountX->close();
    platformMountY->close();
    platformMountZ->close();
    platformMountRoll->close();
    platformMountPitch->close();
    platformMountYaw->close();

    // Close scanner buffers
    scannerPositionX->close();
    scannerPositionY->close();
    scannerPositionZ->close();
    scannerRoll->close();
    scannerPitch->close();
    scannerYaw->close();

    // Close scanner head buffers
    scannerHeadRoll->close();
    scannerHeadPitch->close();
    scannerHeadYaw->close();

    // Close deflector buffers
    deflectorEmittingRoll->close();
    deflectorEmittingPitch->close();
    deflectorEmittingYaw->close();

    // Close beam buffers
    beamOriginX->close();
    beamOriginY->close();
    beamOriginZ->close();
    beamRoll->close();
    beamPitch->close();
    beamYaw->close();

    // Stochastic measurement buffers
    measErrSeq->close();
    measErrPar->close();
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

void HDA_SimStepRecorder::recordPlatformMount(){
    // Obtain platform
    Platform &p = *(sp->getScanner()->platform);
    // Record platform's absolute mount position
    glm::dvec3 const pos = p.getAbsoluteMountPosition();
    platformMountX->push(pos.x);
    platformMountY->push(pos.y);
    platformMountZ->push(pos.z);
    // Record platform's absolute mount attitude
    double roll, pitch, yaw;
    try{
        p.getAbsoluteMountAttitude().getAngles(
            &RotationOrder::XYZ, roll, pitch, yaw
        );
    }
    catch(HeliosException &hex){ // Probably, Gimbal lock
        roll = -PI_2;
        pitch = roll;
        yaw = roll;
        std::stringstream ss;
        ss  << "The following exception occurred when recording the "
            << "platform's absolute mount attitude:\n"
            << hex.what() << "\n"
            << "Recording " << roll << " value instead.";
        logging::WARN(ss.str());
    }
    platformMountRoll->push(roll);
    platformMountPitch->push(pitch);
    platformMountYaw->push(yaw);
}

void HDA_SimStepRecorder::recordScanner(){
    // Obtain scanner
    Scanner &s = *(sp->getScanner());
    // Record scanner position
    glm::dvec3 const pos = s.getHeadRelativeEmitterPosition();
    scannerPositionX->push(pos.x);
    scannerPositionY->push(pos.y);
    scannerPositionZ->push(pos.z);
    // Record scanner angles
    double roll, pitch, yaw;
    s.getHeadRelativeEmitterAttitude().getAngles(
        &RotationOrder::XYZ, roll, pitch, yaw
    );
    scannerRoll->push(roll);
    scannerPitch->push(pitch);
    scannerYaw->push(yaw);
}

void HDA_SimStepRecorder::recordScannerHead(){
    // Obtain scanner's head
    ScannerHead &sh = *(sp->getScanner()->getScannerHead());
    // Record scanner's head angles
    double roll, pitch, yaw;
    sh.getMountRelativeAttitude().getAngles(
        &RotationOrder::XYZ, roll, pitch, yaw
    );
    scannerHeadRoll->push(roll);
    scannerHeadPitch->push(pitch);
    scannerHeadYaw->push(yaw);
}

void HDA_SimStepRecorder::recordDeflector(){
    // Obtain beam deflector
    AbstractBeamDeflector &bd = *(sp->getScanner()->getBeamDeflector());
    // Record deflector's angles
    double roll, pitch, yaw;
    bd.getEmitterRelativeAttitude().getAngles(
        &RotationOrder::XYZ, roll, pitch, yaw
    );
    deflectorEmittingRoll->push(roll);
    deflectorEmittingPitch->push(pitch);
    deflectorEmittingYaw->push(yaw);
}

void HDA_SimStepRecorder::recordBeam(){
    // Obtain scanner
    Platform &p = *(sp->getScanner()->platform);
    Scanner &s = *(sp->getScanner());
    // Record beam origin
    glm::dvec3 bo = p.getAbsoluteMountPosition() +
        s.getHeadRelativeEmitterPosition();
    beamOriginX->push(bo.x);
    beamOriginY->push(bo.y);
    beamOriginZ->push(bo.z);
    // Record beam attitude
    Rotation ba = s.calcAbsoluteBeamAttitude();
    double roll, pitch, yaw;
    ba.getAngles(&RotationOrder::XYZ, roll, pitch, yaw);
    beamRoll->push(roll);
    beamPitch->push(pitch);
    beamYaw->push(yaw);
}

void HDA_SimStepRecorder::recordStochastic(){
    // Obtain scanner and pulse thread pool
    Scanner &s = (*sp->getScanner());
    ScanningPulseProcess * spp = s.getScanningPulseProcess();

    // Obtain sequential randomness generator for measurement error
    std::shared_ptr<RandomnessGenerator<double>> rg1Seq = s.randGen1;

    // Obtain parallel randomness generator
    RandomnessGenerator<double> *rg1Par = nullptr;
    WarehouseScanningPulseProcess * wspp =
        dynamic_cast<WarehouseScanningPulseProcess *>(spp);
    if(wspp != nullptr){
        rg1Par = wspp->pool.randGens;
    }
    else{
        BuddingScanningPulseProcess *bspp =
            dynamic_cast<BuddingScanningPulseProcess *>(spp);
        rg1Par = bspp->pool.randGens;
    }

    // Prepare fake pulse
    SimulatedPulse fakePulse(
        glm::dvec3(0, 0, 0),
        Rotation(),
        0.0,
        0,
        0,
        0
    );

    // Record sequential measurement error
    size_t const N_SAMPLES = 1000000;
    double const ERR_BASE = 1000; // Distance without error
    double err;
    FullWaveformPulseRunnable fwpr(sp->getScanner(), fakePulse);
    fwpr.detector = s.getDetector();
    for(size_t i = 0 ; i < N_SAMPLES ; ++i) {
        err = ERR_BASE;
        fwpr.applyMeasurementError(
            *rg1Seq, err, fakePulse.getOriginRef(), fakePulse.getOriginRef()
        );
        measErrSeq->push(err-ERR_BASE);
    }

    // Record parallel measurement error
    for(size_t i = 0 ; i < N_SAMPLES ; ++i) {
        err = ERR_BASE;
        fwpr.applyMeasurementError(
            *rg1Par, err, fakePulse.getOriginRef(), fakePulse.getOriginRef()
        );
        measErrPar->push(err-ERR_BASE);
    }


}

std::string HDA_SimStepRecorder::craftOutputPath(std::string const &fname){
    std::stringstream ss;
    ss  << outdir
        << boost::filesystem::path::preferred_separator
        << fname;
    return ss.str();
}

#endif