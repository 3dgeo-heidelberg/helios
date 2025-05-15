#pragma once

#include <glm/glm.hpp>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class representing a pulse record. In other words, it contains
 *  information registered during simulation to represent the state of a
 *  pulse.
 *
 * Do not confuse this class with the Pulse class and its derived classes.
 *  These are meant to be used to represent the actual pulse why PulseRecord
 *  just stores a representation of a pulse during execution to be exported
 *  to a file. Also, do not confuse with HDA_PulseRecorder from the
 *  dataanalytics mode or any other HDA class because these are meant for
 *  debugging and analysis purposes, not as a typical output.
 *
 * @see Pulse
 */
class PulseRecord
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The x coordinate of the pulses' origin
   */
  double ox;
  /**
   * @brief The y coordinate of the pulses' origin
   */
  double oy;
  /**
   * @brief The z coordinate of the pulses' origin
   */
  double oz;
  /**
   * @brief The x component of the pulses' director vector
   */
  double vx;
  /**
   * @brief The y component of the pulses' director vector
   */
  double vy;
  /**
   * @brief The z component of the pulses' director vector
   */
  double vz;
  /**
   * @brief The GPS time when the pulse was emitted (in nanoseconds)
   */
  double time;
  /**
   * @brief The index representing the pulse
   */
  int pulseIndex;
  /**
   * @brief The index representing the device that emitted the pulse
   * @see ScanningDevice
   */
  int deviceIndex;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Instantiate a pulse record from given basic data
   */
  PulseRecord(double const ox,
              double const oy,
              double const oz,
              double const vx,
              double const vy,
              double const vz,
              double const time,
              int const pulseIndex,
              int const deviceIndex)
    : ox(ox)
    , oy(oy)
    , oz(oz)
    , vx(vx)
    , vy(vy)
    , vz(vz)
    , time(time)
    , pulseIndex(pulseIndex)
    , deviceIndex(deviceIndex)
  {
  }
  /**
   * @brief Instantiate a pulse record from given GLM and basic data
   * @param origin The vector representing the pulses' origin
   * @param dir The vector representing the pulses' director vector
   */
  PulseRecord(glm::dvec3 const origin,
              glm::dvec3 const dir,
              double const time,
              int const pulseIndex,
              int const deviceIndex)
    : PulseRecord(origin.x,
                  origin.y,
                  origin.z,
                  dir.x,
                  dir.y,
                  dir.z,
                  time,
                  pulseIndex,
                  deviceIndex)
  {
  }
  /**
   * @brief Default constructor for PulseRecord
   */
  PulseRecord()
    : PulseRecord(0, 0, 0, 0, 0, 0, -1, -1, -1)
  {
  }
  virtual ~PulseRecord() = default;
};
