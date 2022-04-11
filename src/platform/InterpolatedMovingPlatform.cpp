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
    InterpolationScope scope
) :
    MovingPlatform(),
    stepLoop(stepLoop),
    scope(scope),
    timeFrontiers(ddm.getTimeVector()),
    frontierValues(tdm.getX().rows(0, tdm.getX().n_rows-2)),
    frontierDerivatives(ddm.getA())
{
    // Build DesignTrajectoryFunction
    tf = std::make_shared<DesignTrajectoryFunction>(
        timeFrontiers,
        frontierValues,
        frontierDerivatives
    );
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
                setAttitude(
                    Rotation(Directions::right, x[0]).applyTo(
                        Rotation(Directions::forward, x[1])
                    ).applyTo(
                        Rotation(Directions::up, x[2])
                    )
                );
            };
            break;
        case InterpolationScope::POSITION_AND_ATTITUDE:
            doStepUpdates = [&] (double const t) -> void{
                arma::Col<double> const x = tf->eval(t); //roll,pitch,yaw,x,y,z
                setPosition(glm::dvec3(x[3], x[4], x[5]));
                setAttitude(
                    Rotation(Directions::right, x[0]).applyTo(
                        Rotation(Directions::forward, x[1])
                    ).applyTo(
                        Rotation(Directions::up, x[2])
                    )
                );
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
    // Waypoint reached if distance(platform, waypoint) < 1cm
    return glm::l2Norm(cached_vectorToTarget) < 0.01;
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
