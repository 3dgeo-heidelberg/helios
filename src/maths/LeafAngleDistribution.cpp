#include "LeafAngleDistribution.h"
#include "MathConstants.h"

#include <HeliosException.h>

#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {

using DensityFunction = std::function<double(double)>;

/**
 * Named distributions of de Wit (1965), as densities of the leaf
 * inclination over [0, pi/2]
 */
std::map<std::string, DensityFunction> const&
namedDensities()
{
  static std::map<std::string, DensityFunction> const densities{
    { "planophile",
      [](double t) { return 2.0 / PI * (1.0 + std::cos(2.0 * t)); } },
    { "erectophile",
      [](double t) { return 2.0 / PI * (1.0 - std::cos(2.0 * t)); } },
    { "plagiophile",
      [](double t) { return 2.0 / PI * (1.0 - std::cos(4.0 * t)); } },
    { "extremophile",
      [](double t) { return 2.0 / PI * (1.0 + std::cos(4.0 * t)); } },
    { "spherical", [](double t) { return std::sin(t); } },
    { "uniform", [](double) { return 2.0 / PI; } },
  };
  return densities;
}

std::vector<double>
binCentres()
{
  double const w = PI_HALF / LeafAngleDistribution::INCLINATION_BINS;
  std::vector<double> c(LeafAngleDistribution::INCLINATION_BINS);
  for (std::size_t i = 0; i < c.size(); ++i)
    c[i] = (static_cast<double>(i) + 0.5) * w;
  return c;
}

} // namespace

// ***  CONSTRUCTION  *** //
// ********************** //
LeafAngleDistribution::LeafAngleDistribution(
  std::vector<double> const& inclinations_rad,
  std::vector<double> const& density,
  std::string name)
  : name(std::move(name))
{
  if (inclinations_rad.size() != density.size())
    throw HeliosException(
      "Leaf angle distribution: inclinations and density differ in size");
  if (inclinations_rad.size() < 2)
    throw HeliosException(
      "Leaf angle distribution: at least two values are required");
  for (std::size_t i = 0; i < inclinations_rad.size(); ++i) {
    double const t = inclinations_rad[i];
    if (!std::isfinite(t) || t < 0.0 || t > PI_HALF)
      throw HeliosException("Leaf angle distribution: inclinations must be "
                            "within [0, 90] degrees");
    if (i > 0 && t <= inclinations_rad[i - 1])
      throw HeliosException("Leaf angle distribution: inclinations must be "
                            "strictly increasing");
    if (!std::isfinite(density[i]) || density[i] < 0.0)
      throw HeliosException(
        "Leaf angle distribution: density values must be non-negative");
  }

  // Linear interpolation onto the bin centres, constant beyond the ends
  std::vector<double> const centres = binCentres();
  std::vector<double> binDensity(centres.size());
  for (std::size_t i = 0; i < centres.size(); ++i) {
    double const t = centres[i];
    auto const it =
      std::upper_bound(inclinations_rad.begin(), inclinations_rad.end(), t);
    if (it == inclinations_rad.begin()) {
      binDensity[i] = density.front();
    } else if (it == inclinations_rad.end()) {
      binDensity[i] = density.back();
    } else {
      std::size_t const j = std::distance(inclinations_rad.begin(), it);
      double const t0 = inclinations_rad[j - 1];
      double const t1 = inclinations_rad[j];
      double const f = (t - t0) / (t1 - t0);
      binDensity[i] = (1.0 - f) * density[j - 1] + f * density[j];
    }
  }
  build(std::move(binDensity));
}

LeafAngleDistribution
LeafAngleDistribution::fromName(std::string const& name)
{
  std::string const key = boost::algorithm::to_lower_copy(name);
  auto const it = namedDensities().find(key);
  if (it == namedDensities().end()) {
    std::stringstream ss;
    ss << "Unknown leaf angle distribution \"" << name
       << "\". Expected one of: planophile, erectophile, plagiophile, "
          "extremophile, spherical, uniform, or the path to a file.";
    throw HeliosException(ss.str());
  }
  // Evaluated at the bin centres, so the interpolation is exact
  std::vector<double> const centres = binCentres();
  std::vector<double> binDensity(centres.size());
  for (std::size_t i = 0; i < centres.size(); ++i)
    binDensity[i] = it->second(centres[i]);
  return LeafAngleDistribution(centres, binDensity, key);
}

LeafAngleDistribution
LeafAngleDistribution::fromFile(std::string const& path)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    std::stringstream ss;
    ss << "Failed to open leaf angle distribution file \"" << path << "\"";
    throw HeliosException(ss.str());
  }
  std::vector<double> inclinations_rad, density;
  std::string line;
  std::size_t lineNumber = 0;
  while (std::getline(ifs, line)) {
    ++lineNumber;
    boost::algorithm::trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    std::vector<std::string> parts;
    boost::split(parts, line, boost::is_any_of(","));
    try {
      if (parts.size() != 2)
        throw std::invalid_argument("expected two values");
      inclinations_rad.push_back(std::stod(parts[0]) * PI / 180.0);
      density.push_back(std::stod(parts[1]));
    } catch (std::exception const&) {
      std::stringstream ss;
      ss << "Invalid line " << lineNumber << " in leaf angle distribution "
         << "file \"" << path << "\". Expected \"inclination_deg,g_L\".";
      throw HeliosException(ss.str());
    }
  }
  return LeafAngleDistribution(inclinations_rad, density, path);
}

bool
LeafAngleDistribution::isNamedType(std::string const& name)
{
  return namedDensities().count(boost::algorithm::to_lower_copy(name)) > 0;
}

// ***  METHODS  *** //
// ***************** //
double
LeafAngleDistribution::G(glm::dvec3 const& direction) const
{
  double const len = glm::length(direction);
  if (!(len > 0.0))
    throw HeliosException(
      "Leaf angle distribution: direction must not be zero");
  double const cosZenith = std::min(1.0, std::abs(direction.z) / len);
  return G(std::acos(cosZenith));
}

double
LeafAngleDistribution::G(double zenith_rad) const
{
  // G(zeta) = G(pi - zeta): only the angle to the vertical axis matters
  double zeta = std::abs(zenith_rad);
  if (zeta > PI_HALF)
    zeta = PI - zeta;
  zeta = std::clamp(zeta, 0.0, PI_HALF);
  double const x = zeta / PI_HALF * static_cast<double>(G_SAMPLES - 1);
  std::size_t const i =
    std::min(static_cast<std::size_t>(x), G_SAMPLES - 2);
  double const f = x - static_cast<double>(i);
  return (1.0 - f) * gTable[i] + f * gTable[i + 1];
}

double
LeafAngleDistribution::sampleIncidenceAngle(glm::dvec3 const& direction,
                                            NoiseSource<double>& uniform) const
{
  double const len = glm::length(direction);
  if (!(len > 0.0))
    throw HeliosException(
      "Leaf angle distribution: direction must not be zero");
  glm::dvec3 const d = direction / len;
  double const w = binWidth();
  for (std::size_t attempt = 0; attempt < MAX_SAMPLING_ATTEMPTS; ++attempt) {
    // Leaf inclination from g_L (inverse CDF, uniform within the bin)
    double const u = std::clamp(uniform.next(), 0.0, 1.0);
    auto const it = std::lower_bound(cdf.begin(), cdf.end(), u);
    std::size_t const i =
      std::min<std::size_t>(std::distance(cdf.begin(), it), cdf.size() - 1);
    double const lower = (i == 0) ? 0.0 : cdf[i - 1];
    double const p = cdf[i] - lower;
    double const f = (p > 0.0) ? std::clamp((u - lower) / p, 0.0, 1.0) : 0.5;
    double const thetaL = (static_cast<double>(i) + f) * w;

    // Leaf azimuth, uniform
    double const phiL = PI_2 * uniform.next();

    // Accept with probability |Omega' . Omega_L|
    glm::dvec3 const n(std::sin(thetaL) * std::cos(phiL),
                       std::sin(thetaL) * std::sin(phiL),
                       std::cos(thetaL));
    double const c = std::min(1.0, std::abs(glm::dot(d, n)));
    if (uniform.next() < c)
      return std::acos(c);
  }
  std::stringstream ss;
  ss << "Leaf angle distribution \"" << name << "\": no leaf orientation "
     << "accepted after " << MAX_SAMPLING_ATTEMPTS << " attempts. G is "
     << "close to zero for this beam direction.";
  throw HeliosException(ss.str());
}

double
LeafAngleDistribution::psi(double zenith_rad, double inclination_rad)
{
  // Mean of |cos(zeta) cos(thetaL) + sin(zeta) sin(thetaL) cos(phi)|
  // over phi in [0, 2pi)
  double const a =
    std::abs(std::cos(zenith_rad)) * std::cos(inclination_rad);
  double const b =
    std::abs(std::sin(zenith_rad)) * std::sin(inclination_rad);
  if (a >= b)
    return a;
  double const phi0 = std::acos(-a / b);
  return (a * (2.0 * phi0 - PI) + 2.0 * b * std::sin(phi0)) / PI;
}

// ***  INNER METHODS  *** //
// *********************** //
void
LeafAngleDistribution::build(std::vector<double> binDensity)
{
  double const w = binWidth();
  double sum = 0.0;
  for (double const g : binDensity)
    sum += g * w;
  if (!(sum > 0.0) || !std::isfinite(sum))
    throw HeliosException(
      "Leaf angle distribution: density must not be zero everywhere");

  // Normalized density and CDF
  inclinations = binCentres();
  density = std::move(binDensity);
  cdf.resize(density.size());
  double acc = 0.0;
  for (std::size_t i = 0; i < density.size(); ++i) {
    density[i] /= sum;
    acc += density[i] * w;
    cdf[i] = acc;
  }
  cdf.back() = 1.0;

  // G table (midpoint rule over the inclination bins)
  gTable.resize(G_SAMPLES);
  for (std::size_t j = 0; j < G_SAMPLES; ++j) {
    double const zeta =
      PI_HALF * static_cast<double>(j) / static_cast<double>(G_SAMPLES - 1);
    double g = 0.0;
    for (std::size_t i = 0; i < density.size(); ++i)
      g += density[i] * psi(zeta, inclinations[i]) * w;
    gTable[j] = g;
  }
}

double
LeafAngleDistribution::binWidth()
{
  return PI_HALF / static_cast<double>(INCLINATION_BINS);
}
