#include <platform/InterpolatedMovingPlatform.h>
#include <maths/Directions.h>
#include <platform/trajectory/DesignTrajectoryFunction.h>

#include <glm/gtx/norm.hpp>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
InterpolatedMovingPlatform::InterpolatedMovingPlatform(
    SimulationStepLoop &stepLoop,
    TemporalDesignMatrix<double, double> const &tdm,
    DiffDesignMatrix<double, double> const &ddm,
    InterpolationScope scope,
    bool const syncGPSTime,
    double const startTime,
    RotationSpec rotspec
) :
    MovingPlatform(),
    stepLoop(stepLoop),
    scope(scope),
    rotspec(rotspec),
    timeFrontiers(ddm.getTimeVector()),
    frontierValues(tdm.getX().rows(0, tdm.getX().n_rows-2)),
    frontierDerivatives(ddm.getA()),
    syncGPSTime(syncGPSTime),
    startTime(startTime),
    currentLegStartTime(0)
{
    // Build DesignTrajectoryFunction
    tf = std::make_shared<DesignTrajectoryFunction>(
        timeFrontiers,
        frontierValues,
        frontierDerivatives
    );
    // Configure rotation specification
    switch(rotspec){
        case RotationSpec::CANONICAL:
            calcAttitude = [] (arma::Col<double> const x) -> Rotation {
                return Rotation(Directions::right, x[0]).applyTo(
                    Rotation(Directions::forward, x[1])
                ).applyTo(
                    Rotation(Directions::up, x[2])
                );
            };
            _getRollPitchYaw = [] (
                double &roll, double &pitch, double &yaw, Rotation &attitude
            ) -> void {
                attitude.getAngles(&RotationOrder::XYZ, roll, pitch, yaw);
            };
            break;
        case RotationSpec::ARINC_705:
            calcAttitude = [] (arma::Col<double> const x) -> Rotation {
                return Rotation(Directions::right, -x[0]).applyTo(
                    Rotation(Directions::forward, -x[1])
                ).applyTo(
                    Rotation(Directions::up, -x[2])
                );
            };
            _getRollPitchYaw = [] (
                double &roll, double &pitch, double &yaw, Rotation &attitude
            ) -> void {
                attitude.getAngles(&RotationOrder::XYZ, roll, pitch, yaw);
                roll = -roll;
                pitch = -pitch;
                yaw = -yaw;
            };
            break;
        default:
            std::stringstream ss;
            ss << "InterpolatedMovingPlatform::InterpolatedMovingPlatform "
               << "failed to construct because an unexpected RotationSpec "
               << "was given";
            logging::ERR(ss.str());
            std::exit(3);
            break;
    }
    // Configure update function to be computed once at each sim step
    switch(scope){
        case InterpolationScope::POSITION:
            doStepUpdates = [&] (double const t) -> void{
                arma::Col<double> const x = tf->eval(t); // x,y,z
                setPosition(glm::dvec3(x[0], x[1], x[2]));
            };
            break;
        case InterpolationScope::ATTITUDE:
            doStepUpdates = [&] (double const t) -> void{
                arma::Col<double> const x = tf->eval(t); // roll,pitch,yaw
                setAttitude(calcAttitude(x));
            };
            break;
        case InterpolationScope::POSITION_AND_ATTITUDE:
            doStepUpdates = [&] (double const t) -> void{
                arma::Col<double> const x = tf->eval(t); //roll,pitch,yaw,x,y,z
                setPosition(glm::dvec3(x[3], x[4], x[5]));
                setAttitude(calcAttitude(x));
            };
            break;
        default:
            throw HeliosException(
                "InterpolatedMovingPlatform::InterpolatedMovingPlatform "
                "failed to recognize given InterpolationScope"
            );
    }
}

// ***  M E T H O D S  *** //
// *********************** //
void InterpolatedMovingPlatform::doSimStep(int simFrequency_hz){
    doStepUpdates(stepLoop.getCurrentTime());
}
bool InterpolatedMovingPlatform::waypointReached(){
    double const legEndTime = currentLegTimeDiff + currentLegStartTime;
    double const t = stepLoop.getCurrentTime();
    return t >= legEndTime;
}
void InterpolatedMovingPlatform::toTrajectoryTime(double const t){
    double h = t - tf->getFpiem().getT();
    if(h >= 0){ // Advance to next tStart
        tf->getFpiem().eval(h);
    }
    else{ // Restart and advance to next tStart
        tf->getFpiem().restart();
        tf->getPclsf().restart();
        h = t - tf->getFpiem().getT();
        tf->getFpiem().eval(h);
    }
}
