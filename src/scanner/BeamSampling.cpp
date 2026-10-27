#include "BeamSampling.h"
#include <maths/Directions.h>
#include <maths/MathConstants.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

std::string
toString(BeamSamplingMethod method)
{
  /* Use the same method names in configuration and reporting. */
  switch (method) {
    case BeamSamplingMethod::Annular:
      return "annular";
    case BeamSamplingMethod::Cartesian:
      return "cartesian";
  }

  throw std::invalid_argument("Unknown beam sampling method");
}

BeamSamplingMethod
parseBeamSamplingMethod(std::string const& value)
{
  /* Accept only the supported configuration names. */
  if (value == "annular")
    return BeamSamplingMethod::Annular;
  if (value == "cartesian")
    return BeamSamplingMethod::Cartesian;

  throw std::invalid_argument(
    "beamSamplingMethod must be annular or cartesian");
}

void
BeamSamplingSettings::validate() const
{
  /* Validate the method even when settings are constructed directly in C++. */
  toString(method);

  /* Quality determines the ray count; cutoff determines the sampled extent. */
  if (quality < 1)
    throw std::invalid_argument("beamSampleQuality must be >= 1");
  if (!std::isfinite(cutoff) || cutoff <= 0.0)
    throw std::invalid_argument("beamSamplingFactor must be finite and > 0");
}

double
normalizeBeamRotation(double radians)
{
  if (!std::isfinite(radians))
    throw std::invalid_argument("Beam rotation must be finite");

  /* Wrap complete turns while preserving all discrete pattern orientations. */
  double angle = std::fmod(radians, PI_2);
  if (angle < 0.0)
    angle += PI_2;

  // Adding a tiny negative remainder to 2*pi may round up to a full turn.
  return angle == 0.0 || angle == PI_2 ? 0.0 : angle;
}

namespace {
struct TransverseBasis
{
  glm::dvec3 right = Directions::right;
  glm::dvec3 up = Directions::up;
};

// Integral of a normalized one-dimensional Gaussian. Avoid subtracting
// two erf values rounded to one in the tails.
double
gaussianShare(double lo, double hi)
{
  /* Scale normalized coordinates to the error-function arguments. */
  double const a = std::sqrt(2.0) * lo;
  double const b = std::sqrt(2.0) * hi;

  /* Integrate either tail with erfc to avoid cancellation near erf = +/-1. */
  if (lo >= 0.0)
    return 0.5 * (std::erfc(a) - std::erfc(b));
  if (hi <= 0.0)
    return 0.5 * (std::erfc(-b) - std::erfc(-a));

  /* Across the beam axis, the erf values have opposite signs. */
  return 0.5 * (std::erf(b) - std::erf(a));
}

std::vector<Subray>
makeCartesianPattern(int quality,
                     double cutoff,
                     double tx,
                     double ty,
                     TransverseBasis const& basis)
{
  /* An odd number of cells per axis preserves a central ray. Check counts
   * before allocating the quadratic subray table. */
  std::vector<Subray> result;
  auto const q = static_cast<std::size_t>(quality);
  if (q > std::numeric_limits<std::size_t>::max() / 2)
    throw std::invalid_argument("Too many subrays");
  std::size_t const n = 2 * q - 1;
  if (n > result.max_size() / n)
    throw std::invalid_argument("Too many subrays");

  /* Rectangle corners have the largest transverse slope. */
  double const corner = std::hypot(cutoff * tx, cutoff * ty);
  if (!std::isfinite(corner) || std::atan(corner) >= PI / 2.0)
    throw std::invalid_argument("Sampling rectangle is not representable");

  /* Both axes share normalized cell bounds and one-dimensional power shares. */
  struct Interval
  {
    double lo, hi, mid, share;
  };
  std::vector<Interval> intervals;
  intervals.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    double const lo = cutoff * (2.0 * (static_cast<double>(i) / n) - 1.0);
    double const hi = cutoff * (2.0 * (static_cast<double>(i + 1) / n) - 1.0);
    double const mid = i == n / 2 ? 0.0 : lo / 2.0 + hi / 2.0;
    double const share = gaussianShare(lo, hi);
    double const width = hi - lo;

    /* Cell widths and their physical slopes must remain representable. */
    if (!std::isfinite(width) || width <= 0.0 || !std::isfinite(share) ||
        share < 0.0 || !std::isfinite(tx * width) || !std::isfinite(ty * width))
      throw std::invalid_argument("Subray patch is not representable");

    intervals.push_back({ lo, hi, mid, share });
  }

  /* All Cartesian widths are positive: extrema bound every patch area. */
  auto const widths =
    std::minmax_element(intervals.begin(),
                        intervals.end(),
                        [](Interval const& a, Interval const& b) {
                          return a.hi - a.lo < b.hi - b.lo;
                        });
  double const minWidth = widths.first->hi - widths.first->lo;
  double const maxWidth = widths.second->hi - widths.second->lo;
  if ((tx * minWidth) * (ty * minWidth) <= 0.0 ||
      !std::isfinite((tx * maxWidth) * (ty * maxWidth)))
    throw std::invalid_argument("Subray patch is not representable");

  /* Combine axis intervals in row order, placing rays at cell midpoints. */
  result.reserve(n * n);
  for (auto const& y : intervals) {
    for (auto const& x : intervals) {
      double const sx = tx * x.mid, sy = ty * y.mid;
      auto const direction =
        Directions::forward + sx * basis.right + sy * basis.up;
      Rotation const rotation = x.mid == 0.0 && y.mid == 0.0
                                  ? Rotation()
                                  : Rotation(Directions::forward, direction);

      /* Separability gives the cell's power share and transverse patch area. */
      result.push_back(
        { rotation,
          std::atan2(std::hypot(sx, sy), 1.0),
          x.share * y.share,
          (tx * (x.hi - x.lo)) * (ty * (y.hi - y.lo)),
          CartesianCell{ x.mid, y.mid, x.lo, x.hi, y.lo, y.hi } });
    }
  }

  return result;
}

std::vector<Subray>
makeAnnularPattern(int quality,
                   double rhoMax,
                   double tangentX,
                   double tangentY,
                   TransverseBasis const& basis)
{
  /* Build adjacent annuli outward from the central ray. */
  std::vector<Subray> generated;
  double innerRho = 0.0;
  for (int ring = 0; ring < quality; ++ring) {
    /* Determine number of subrays (population) per annulus */
    double const population = (ring == 0) ? 1.0 : std::floor(PI_2 * ring);

    /* Validate population */
    if (population >= static_cast<double>(generated.max_size()))
      throw std::invalid_argument("Too many subrays");
    std::size_t const count = static_cast<std::size_t>(population);
    if (count > generated.max_size() - generated.size())
      throw std::invalid_argument("Too many subrays");

    /* Placement and integration use normalized Gaussian coordinates. */
    double const rho = rhoMax * (ring / (quality - 0.5));
    double const outerRho = (ring == quality - 1)
                              ? rhoMax
                              : rhoMax * ((ring + 0.5) / (quality - 0.5));
    double const extent = (outerRho - innerRho) * (outerRho + innerRho);

    /* Power share per-subray in current annulus */
    double const share =
      std::exp(-2.0 * innerRho * innerRho) * -std::expm1(-2.0 * extent) / count;
    double const areaFactor = PI * tangentX * tangentY * extent / count;
    if (!std::isfinite(share) || !std::isfinite(areaFactor) ||
        areaFactor <= 0.0)
      throw std::invalid_argument("Subray patch is not representable");

    /* Determine direction of each subray */
    for (std::size_t j = 0; j < count; ++j) {
      double const phase = PI_2 * j / count;
      double const u = rho * tangentX * std::sin(phase);
      double const v = rho * tangentY * std::cos(phase);
      double const angle = std::atan2(std::hypot(u, v), 1.0);
      glm::dvec3 const direction =
        Directions::forward + u * basis.right + v * basis.up;
      Rotation const rotation =
        ring == 0 ? Rotation() : Rotation(Directions::forward, direction);

      generated.push_back({ rotation,
                            angle,
                            share,
                            areaFactor,
                            AnnularCell{ rho, innerRho, outerRho } });
    }

    /* The next annulus starts at this annulus's outer boundary. */
    innerRho = outerRho;
  }

  return generated;
}

} // namespace

std::vector<Subray>
makeSubrayPattern(BeamGeometry const& beam,
                  BeamSamplingSettings const& settings)
{
  settings.validate();
  double const rotation = normalizeBeamRotation(beam.rotation_rad);

  /* Validate beamDivergence */
  for (double divergence : { beam.divergenceX_rad, beam.divergenceY_rad }) {
    if (!std::isfinite(divergence) || divergence <= 0.0 || divergence >= PI)
      throw std::invalid_argument(
        "Full beam divergences must be finite and in (0, pi)");
  }
  int const quality = settings.quality;

  /* 1/e^2 tangents; generally different in x and y directions*/
  double const tangentX = std::tan(beam.divergenceX_rad / 2.0);
  double const tangentY = std::tan(beam.divergenceY_rad / 2.0);

  /* Validate 1/e^2 tangents */
  for (double tangent : { tangentX, tangentY }) {
    double const cutoff = settings.cutoff * tangent;
    if (!std::isfinite(cutoff) || cutoff <= 0.0 ||
        std::atan(cutoff) >= PI / 2.0)
      throw std::invalid_argument("Sampling cone is not representable");
  }

  /* Rotate physical slopes after scaling by the principal-axis divergences.
   * Keep the original basis for zero rotation to preserve existing results. */
  TransverseBasis basis;
  if (rotation != 0.0) {
    double const cosine = std::cos(rotation);
    double const sine = std::sin(rotation);
    basis.right = cosine * Directions::right + sine * Directions::up;
    basis.up = -sine * Directions::right + cosine * Directions::up;
  }

  if (settings.method == BeamSamplingMethod::Cartesian)
    return makeCartesianPattern(
      quality, settings.cutoff, tangentX, tangentY, basis);

  return makeAnnularPattern(
    quality, settings.cutoff, tangentX, tangentY, basis);
}
