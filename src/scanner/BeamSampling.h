#pragma once

#include <maths/Rotation.h>
#include <string>
#include <variant>
#include <vector>

enum class BeamSamplingMethod
{
  Annular,
  Cartesian
};
std::string
toString(BeamSamplingMethod method);
BeamSamplingMethod
parseBeamSamplingMethod(std::string const& value);

/** Normalize a finite beam rotation to [0, 2*pi), including positive zero. */
double
normalizeBeamRotation(double radians);

struct BeamGeometry
{
  double divergenceX_rad; // Full divergence angles.
  double divergenceY_rad;
  double rotation_rad = 0.0; // Positive rotation moves local right toward up.
};

struct BeamSamplingSettings
{
  BeamSamplingMethod method = BeamSamplingMethod::Annular;
  int quality = 3;
  double cutoff = 2.0;
  void validate() const;
};

struct BeamSamplingKey
{
  BeamGeometry beam;
  BeamSamplingSettings settings;
  bool operator==(BeamSamplingKey const& other) const
  {
    /* Every input affecting geometry or power shares belongs to the cache key.
     */
    return beam.divergenceX_rad == other.beam.divergenceX_rad &&
           beam.divergenceY_rad == other.beam.divergenceY_rad &&
           beam.rotation_rad == other.beam.rotation_rad &&
           settings.method == other.settings.method &&
           settings.quality == other.settings.quality &&
           settings.cutoff == other.settings.cutoff;
  }
};

// Cell metadata stays in the unrotated principal-axis coordinate system.
struct AnnularCell
{
  double rho;
  double innerRho;
  double outerRho;
};

// Bounds and sample coordinates in normalized transverse slopes.
struct CartesianCell
{
  double u, v;
  double uMin, uMax, vMin, vMax;
};

using SubrayCell = std::variant<AnnularCell, CartesianCell>;
struct Subray
{
  Rotation rotation;
  double angle_rad;
  // Fraction of whole-beam Gaussian power, without renormalization.
  double share;
  double areaFactor; // Transverse patch area per squared axial range.
  SubrayCell cell;
};

std::vector<Subray>
makeSubrayPattern(BeamGeometry const& beam,
                  BeamSamplingSettings const& settings);
