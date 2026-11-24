#include <DetailedVoxel.h>
#include <LeafAngleDistribution.h>
#include <boost/algorithm/string.hpp>

// ***  CONSTRUCTION  *** //
// ********************** //
Primitive*
DetailedVoxel::clone()
{
  DetailedVoxel* dv = new DetailedVoxel();
  _clone(dv);
  return dv;
}
void
DetailedVoxel::_clone(Primitive* p)
{
  Voxel::_clone(p);
  DetailedVoxel* dv = (DetailedVoxel*)p;
  dv->maxPad = maxPad;
  for (size_t i = 0; i < intValues.size(); i++) {
    dv->intValues.push_back(intValues[i]);
  }
  for (size_t i = 0; i < doubleValues.size(); i++) {
    dv->doubleValues.push_back(doubleValues[i]);
  }
}

// ***  M E T H O D S  *** //
// *********************** //
IntersectionHandlingResult
DetailedVoxel::onRayIntersection(NoiseSource<double>& uniformNoiseSource,
                                 glm::dvec3& rayDirection,
                                 glm::dvec3 const& insideIntersectionPoint,
                                 glm::dvec3 const& outsideIntersectionPoint)
{
  std::string const& mode = part->onRayIntersectionMode;
  if (mode == "FIXED") {
    return onRayIntersectionFixed(uniformNoiseSource,
                                  rayDirection,
                                  insideIntersectionPoint,
                                  outsideIntersectionPoint,
                                  part->onRayIntersectionArgument);
  } else if (mode == "SCALED") {
    return onRayIntersectionScaled(uniformNoiseSource,
                                   rayDirection,
                                   insideIntersectionPoint,
                                   outsideIntersectionPoint,
                                   part->onRayIntersectionArgument);
  }

  // By default: TRANSMITTIVE (see isTransmittive)
  return onRayIntersectionTransmittive(uniformNoiseSource,
                                       rayDirection,
                                       insideIntersectionPoint,
                                       outsideIntersectionPoint);
};

IntersectionHandlingResult
DetailedVoxel::onRayIntersectionTransmittive(
  NoiseSource<double>& uniformNoiseSource,
  glm::dvec3& rayDirection,
  glm::dvec3 const& insideIntersectionPoint,
  glm::dvec3 const& outsideIntersectionPoint)
{
  double intersectionLength =
    glm::distance(insideIntersectionPoint, outsideIntersectionPoint);
  double const kappa = computeExtinctionCoefficient(rayDirection);

  // Let the ray continue whenever kappa is nan or 0
  if (isnan(kappa) || kappa == 0.0) {
    return IntersectionHandlingResult(insideIntersectionPoint, true);
  }

  // Probabilistic behavior: free path length following Beer-Lambert
  double const rndProb = uniformNoiseSource.next();
  double const s = -std::log(rndProb) / kappa;
  if (s > intersectionLength) {
    return IntersectionHandlingResult(insideIntersectionPoint, true);
  }

  // Hit: incidence angle between the ray and the leaf it hits, sampled from
  // the leaf angle distribution
  glm::dvec3 const nip = insideIntersectionPoint + rayDirection * s;
  double const incidenceAngle =
    getLeafAngleDistribution().sampleIncidenceAngle(rayDirection,
                                                    uniformNoiseSource);
  return IntersectionHandlingResult(nip, false, incidenceAngle);
}

IntersectionHandlingResult
DetailedVoxel::onRayIntersectionScaled(
  NoiseSource<double>& uniformNoiseSource,
  glm::dvec3& rayDirection,
  glm::dvec3 const& insideIntersectionPoint,
  glm::dvec3 const& outsideIntersectionPoint,
  double scaleFactor)
{
  // Implemented as Fixed mode, might change in the future
  double transmittance = (*this)["transmittance"];
  return IntersectionHandlingResult(
    insideIntersectionPoint, transmittance == 1.0 || isnan(transmittance));
}

IntersectionHandlingResult
DetailedVoxel::onRayIntersectionFixed(
  NoiseSource<double>& uniformNoiseSource,
  glm::dvec3& rayDirection,
  glm::dvec3 const& insideIntersectionPoint,
  glm::dvec3 const& outsideIntersectionPoint,
  double fixedSize)
{
  double transmittance = (*this)["transmittance"];
  return IntersectionHandlingResult(
    insideIntersectionPoint, transmittance == 1.0 || isnan(transmittance));
}

void
DetailedVoxel::onFinishLoading(NoiseSource<double>& uniformNoiseSource)
{
  if (part != nullptr && part->onRayIntersectionMode == "SCALED") {
    double newHalfSize = halfSize * std::pow((*this)["PadBVTotal"] / maxPad,
                                             part->onRayIntersectionArgument);

    if (part->randomShift) {
      // lw: the center may be shifted in one direction by max. half the voxel
      // size minus half the cube size,
      //     so that it still stays within the original boundaries
      double maxShift = halfSize - newHalfSize;
      if (std::isnan(maxShift))
        maxShift = 0;
      glm::dvec3 shift(
        uniformNoiseSource.next() *
          maxShift, // noise is between [-1, 1] as defined in Scene.cpp:65
        uniformNoiseSource.next() * maxShift,
        uniformNoiseSource.next() * maxShift);
      v.pos += shift;
    }

    halfSize = newHalfSize;
    update();
  }
}

// ***  EXTINCTION  *** //
// ******************** //
bool
DetailedVoxel::isTransmittive() const
{
  return isTransmittiveMode(part != nullptr ? part->onRayIntersectionMode
                                            : std::string());
}

bool
DetailedVoxel::isTransmittiveMode(std::string const& mode)
{
  // Consistent with onRayIntersection: any mode other than FIXED and SCALED
  return mode != "FIXED" && mode != "SCALED";
}

LeafAngleDistribution const&
DetailedVoxel::getLeafAngleDistribution() const
{
  static LeafAngleDistribution const defaultLad =
    LeafAngleDistribution::fromName("spherical");
  if (part != nullptr && part->lad != nullptr)
    return *part->lad;
  return defaultLad;
}

double
DetailedVoxel::computeExtinctionCoefficient(glm::dvec3 const& direction)
{
  return (*this)["PadBVTotal"] * getLeafAngleDistribution().G(direction);
}
