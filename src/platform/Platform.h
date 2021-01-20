#pragma once


#include <fstream>

#include <glm/glm.hpp>

#include "Asset.h"
#include "PlatformSettings.h"
#include "maths/Rotation.h"
#include "Scene.h"
#include <NoiseSource.h>
#include "Directions.h"
#ifdef PYTHON_BINDING
#include <PythonDVec3.h>
#endif

/**
 * @brief Class representing a platform asset
 */
class Platform : public Asset {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
	// platform configuration
	/**
	 * @brief Device mount position relative to the platform
	 */
	glm::dvec3 cfg_device_relativeMountPosition = glm::dvec3(0, 0, 0);
	/**
	 * @brief Device mount attitude relative to the platform
	 */
	Rotation cfg_device_relativeMountAttitude = Rotation(glm::dvec3(0, 1, 0), 0);

    // misc stuff
    /**
     * @brief Not used at the moment. Might be removed in the future.
     */
	double lastCheckZ = 0;
    /**
     * @brief Not used at the moment. Might be removed in the future.
     */
	glm::dvec3 lastGroundCheck = glm::dvec3(0, 0, 0);
	/**
	 * @brief Scene where the platform belongs to
	 */
	std::shared_ptr<Scene> scene = nullptr;

	// Noise generators
	/**
	 * @brief Noise source for x component of platform position
	 */
	std::shared_ptr<NoiseSource<double>> positionXNoiseSource = nullptr;
	/**
	 * @brief Noise source for y component of platform position
	 */
    std::shared_ptr<NoiseSource<double>> positionYNoiseSource = nullptr;
    /**
     * @brief Noise source for z component of platform position
     */
    std::shared_ptr<NoiseSource<double>> positionZNoiseSource = nullptr;
    /**
     * @brief Noise source for x component of platform attitude
     */
    std::shared_ptr<NoiseSource<double>> attitudeXNoiseSource = nullptr;
    /**
     * @brief Noise source for y component of platform attitude
     */
    std::shared_ptr<NoiseSource<double>> attitudeYNoiseSource = nullptr;
    /**
     * @brief Noise source for z component of platform attitude
     */
    std::shared_ptr<NoiseSource<double>> attitudeZNoiseSource = nullptr;

	// Output file writer stuff
    /**
     * @brief Not used at the moment. Might be removed in the future.
     */
	double dmax = std::numeric_limits<double>::max();
    /**
     * @brief Not used at the moment. Might be removed in the future.
     */
	glm::dvec3 prevWrittenPos = glm::dvec3(dmax, dmax, dmax);

	// Platform Settings
	/**
	 * @brief How meters per seconds the platform moves. NOTICE this behavior
	 * must be overridden by platforms implementing its own physics model.
	 */
	double cfg_settings_movePerSec_m = 0;
	/**
	 * @brief Origin waypoint
	 */
	glm::dvec3 originWaypoint = glm::dvec3(0, 0, 0);
	/**
	 * @brief Target waypoint (destination)
	 */
	glm::dvec3 targetWaypoint = glm::dvec3(0, 0, 0);
    /**
     * @brief Waypoint after target. For the last target, waypoint after target
     * is equal to the target itself.
     */
	glm::dvec3 nextWaypoint = glm::dvec3(0, 0, 0);

	/**
	 * @brief Flag to specify if the platform must be placed on ground (true)
	 * or not (false)
	 */
	bool onGround = false;
	/**
	 * @brief Flag to specify if platform must work in stop and turn mode
	 * (true) or not (false). Not all platforms support this mode, so it will
	 * only be used when possible.
	 */
	bool stopAndTurn = false;
	/**
	 * @brief Flag to specify if platform must work in smooth turn mode (true)
	 * or not (false). Not all platforms support this mode, so it will only be
	 * used when possible.
	 */
	bool smoothTurn = false;
	/**
	 * @brief Flag to specify if slowdown stage must be enabled (true) or not
	 * (false). Not all platforms have a slowdown stage, so this flag will only
	 * be applied when it is necessary.
	 */
	bool slowdownEnabled = true;

	// State variables
	/**
	 * @brief Platform 3D position
	 */
	glm::dvec3 position = glm::dvec3(0, 0, 0);
	/**
	 * @brief Platform 3D attitude
	 */
	Rotation attitude = Rotation(Directions::up, 0);

    /**
     * @brief Not used at the moment. Might be removed in the future.
     */
	bool mSetOrientationOnLegInit = false;
	/**
	 * @brief Flag to specify if next trajectory needs to be written (true)
	 *  or not (false)
	 */
	bool writeNextTrajectory = true;


	// ***  CACHE ATTRIBUTES  *** //
	// ************************** //
	/**
	 * @brief Cached absolute mount position
	 */
	glm::dvec3 cached_absoluteMountPosition = glm::dvec3(0, 0, 0);
	/**
	 * @brief Cached absolute mount attitude
	 */
	Rotation cached_absoluteMountAttitude = Rotation(glm::dvec3(0, 1, 0), 0);

	/**
	 * @brief Current director vector over XY plane
	 */
	glm::dvec3 cached_dir_current = glm::dvec3(0, 0, 0);
	/**
	 * @brief Current director vector over XY plane (z is always 0)
	 */
	glm::dvec3 cached_dir_current_xy = glm::dvec3(0, 0, 0);

	/**
	 * @brief Distance vector from current position to target
	 */
	glm::dvec3 cached_vectorToTarget = glm::dvec3(0, 0, 0);
	/**
	 * @brief Distance vector from current position to target over XY plane
	 * (z is always 0)
	 */
	glm::dvec3 cached_vectorToTarget_xy = glm::dvec3(0, 0, 0);

	/**
	 * @brief Distance on XY plane between current position and target
	 */
	double cached_distanceToTarget_xy = 0;

	/**
	 * @brief Director vector from origin to target over the XY plane
	 * (z is always 0).
	 */
	glm::dvec3 cached_originToTargetDir_xy = glm::dvec3(0, 0, 0);
	/**
	 * @brief Director vector from target to after target waypoint over the
	 * XY plane (z is always 0).
	 */
	glm::dvec3 cached_targetToNextDir_xy = glm::dvec3(0, 0, 0);
	/**
	 * @brief Angle between director vector from origin to target and director
	 * vector from target to waypoint after target
	 */
	double cached_endTargetAngle_xy;
	/**
	 * @brief Angle between current director vector and director vector from
	 * target to waypoint after target
	 */
	double cached_currentAngle_xy;
    /**
     * @brief Angle in \f$[0, 2\pi)\f$ which identifies director vector from
     * origin waypoint to target waypoint
     */
    double cached_originToTargetAngle_xy;
	/**
	 * @brief Angle in \f$[0, 2\pi)\f$ which identifies director vector from
	 * target waypoint to waypoint after target
	 */
	double cached_targetToNextAngle_xy;


	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Default platform constructor
	 */
	Platform() = default;
	virtual std::shared_ptr<Platform> clone();
	virtual void _clone(std::shared_ptr<Platform> p);


	// ***  M E T H O D S  *** //
	// *********************** //
	/**
	 * @brief Prepare the platform to deal with simulation
	 * @param simFrequency_hz Simulation frequency the platform will work with
	 */
	virtual void prepareSimulation(int simFrequency_hz) {updateStaticCache();}
	/**
	 * @brief Apply given platform settings to the platform
	 * @param settings Settings to be applied to the platform
	 * @param manual Not used by base Platform class.
	 */
	virtual void applySettings(
	    std::shared_ptr<PlatformSettings> settings,
	    bool manual
    );

	/**"
	 * @brief Cache update which only needs to be performed after static
	 * modifications. Updating static cache also updates dynamic cache.
	 * @see Platform::updateDynamicCache
	 */
	virtual void updateStaticCache();
	/**
	 * @brief Cache update which must be performed between simulation steps
	 * and after static modifications. Updating static cache also updates
	 * dynamic cache but updating dynamic cache does not update static cache.
	 * @see Platform::updateStaticCache
	 */
	void updateDynamicCache();
	/**
	 * @brief Check if platform has reached its destination way point (true)
	 * or not (false)
	 * @return True if platform destination way point has been reached (true)
	 * or not (false)
	 */
	virtual bool waypointReached();

	/**
	 * @brief Do corresponding computations for the platform at current
	 * simulation step
	 * @param simFrequency_hz Simulation frequency
	 */
	virtual void doSimStep(int simFrequency_hz){}

    /**
     * @brief Function to initialize leg when working in manual mode
     */
    virtual void initLegManual() {}

    /**
     * @brief Function to initialize leg when working in default mode
     */
    virtual void initLeg() {}


    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Set platform attitude
     * @param attitude New attitude for the platform
     */
    virtual void setAttitude(Rotation attitude);
    /**
     * @brief Set platform origin way point
     * @param origin New origin way point
     */
    virtual void setOrigin(glm::dvec3 origin);
    /**
     * @brief Set platform destination way point
     * @param dest New destination way point
     */
    virtual void setDestination(glm::dvec3 dest);
    /**
     * @brief Set platform after destination way point
     * @param next New after destination way point
     */
    virtual void setAfterDestination(glm::dvec3 next);
    /**
     * @brief Set platform position
     * @param pos New position for the platform
     */
    void setPosition(glm::dvec3 pos);
    /**
     * @brief Set platform position
     * @param pos New position for the platform
     */
    void setOffset(glm::dvec3 pos);

	/**
	 * @brief Obtain platform absolute mount attitude
	 * @return Platform absolute mount attitude
	 */
	Rotation getAbsoluteMountAttitude() {
		return cached_absoluteMountAttitude;
	}

	/**
	 * @brief Obtain platform absolute mount position
	 * @return Platform absolute mount position
	 */
	glm::dvec3 getAbsoluteMountPosition() {
		return cached_absoluteMountPosition;
	}

	/**
	 * @brief Obtain platform attitude
	 * @return Platform attitude
	 */
	Rotation getAttitude() {
		return this->attitude;
	}
	/**
	 * @brief Obtain the directional attitude.
	 * While the attitude represents the platform orientation, the directional
	 * attitude represents the movement direction
	 *
	 * By default, attitude and directional attitude are the same.
	 * But certain platform types might require to consider both attitudes
	 * separately.
	 */
	virtual Rotation getDirectionalAttitude(){
        return this->attitude;
	}
	/**
	 * @brief Obtain platform current direction
	 */
	virtual glm::dvec3 getCurrentDirection(){
        return getDirectionalAttitude().applyTo(Directions::forward);
	}

	/**
	 * @brief Obtain platform position
	 * @return Platform position
	 */
	glm::dvec3 getPosition() {
		return position;
	}

	/**
	 * @brief Obtain platform vector to target (cache)
	 * @return Platform vector to target (cache)
	 */
	glm::dvec3 getVectorToTarget() {
		return cached_vectorToTarget;
	}

	/**
	 * @brief Obtain platform roll, pitch and yaw angles. Notice not all
	 * platforms track those angles
	 * @param[out] roll Used to return roll angle
	 * @param[out] pitch Used to return pitch angle
	 * @param[out] yaw Used to return yaw angle
	 */
	virtual inline void getRollPitchYaw(
	    double &roll, double &pitch, double &yaw
    ) {attitude.getAngles(&RotationOrder::XYZ, roll, pitch, yaw);}

    /**
     * @brief Set the heading angle in radians. This angle can be understood
     * as yaw in most cases
     * @param rad New heading angle in radians
     */
	virtual void setHeadingRad(double rad) {}
	/**
	 * @brief Obtain platform heading angle in radians, which can be understood
	 * as yaw in most cases
	 * @return Platform heading angle in radians
	 */
	virtual double getHeadingRad() {return 0;}

	/**
	 * @brief Obtain platform velocity vector
	 * @return Platform velocity vector
	 */
    virtual glm::dvec3 getVelocity() {return glm::dvec3(0,0,0);}
    /**
     * @brief Check if platform can move (true) or not (false)
     * @return True if platform can move, false otherwise
     */
    virtual bool canMove() {return false;}
    /**
     * @brief Check if platform support stop and turn mode (true) or not
     *  (false)
     * @return True if platform support stop and turn mode, false otherwise
     */
    virtual bool canStopAndTurn() {return false;}

};