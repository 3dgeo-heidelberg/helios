#include "Voxel.h"
#include <MathConstants.h>
#include <glm/gtx/vector_angle.hpp>

// ***  CONSTRUCTION  *** //
// ********************** //
Voxel::Voxel(glm::dvec3 center, double voxelSize) {
	v.pos = center;
	this->halfSize = voxelSize / 2;
	update();
}

Voxel::Voxel(double x, double y, double z, double halfVoxelSize){
    v.pos = glm::dvec3(x, y, z);
    this->halfSize = halfVoxelSize;
    update();
}
Primitive* Voxel::clone(){
    Voxel *vox = new Voxel();
    _clone(vox);
    return vox;
}
void Voxel::_clone(Primitive *p){
    Primitive::_clone(p);
    Voxel *vox = (Voxel *) p;
    vox->v = Vertex(this->v);
    vox->numPoints = numPoints;
    vox->r = r;
    vox->g = g;
    vox->b = b;
    vox->bbox = new AABB(*bbox);
    vox->color = Color4f(color);
    vox->halfSize = halfSize;
}

// ***  M E T H O D S  *** //
// *********************** //
AABB* Voxel::getAABB() {
	return bbox;
}

glm::dvec3 Voxel::getCentroid() {
	return v.pos;
}

double Voxel::getIncidenceAngle_rad(
    const glm::dvec3& rayOrigin,
    const glm::dvec3& rayDir,
    const glm::dvec3& intersectionPoint
){
    // If there is no valid normal, use closest face strategy
    if(!hasNormal()){
        return getIncidenceAngleClosestFace_rad(
            rayOrigin,
            rayDir,
            intersectionPoint
        );
    }

    // Determine incidence angle considering voxel normal
    double const angle = glm::angle(v.normal, rayDir);
    return (angle > PI_HALF) ? M_PI - angle : angle;  // Return min. angle
}

double Voxel::getIncidenceAngleClosestFace_rad(
    const glm::dvec3& rayOrigin,
    const glm::dvec3& rayDir,
    const glm::dvec3& intersectionPoint
){
    // TODO Pending : Use dot^2 instead of distance should be more efficient
    // Determine normal of closest face to compute incidence angle
    glm::dvec3 normal(1, 0, 0);
    glm::dvec3 fc = v.pos + glm::dvec3(halfSize, 0, 0);
    double minDist = glm::distance(fc, intersectionPoint);
    fc = v.pos + glm::dvec3(-halfSize, 0, 0);
    double dist = glm::distance(fc, intersectionPoint);
    if(dist < minDist){
        minDist = dist;
        normal = glm::dvec3(-1, 0, 0);
    }
    fc = v.pos + glm::dvec3(0, halfSize, 0);
    dist = glm::distance(fc, intersectionPoint);
    if(dist < minDist){
        minDist = dist;
        normal = glm::dvec3(0, 1, 0);
    }
    fc = v.pos + glm::dvec3(0, -halfSize, 0);
    dist = glm::distance(fc, intersectionPoint);
    if(dist < minDist){
        minDist = dist;
        normal = glm::dvec3(0, -1, 0);
    }
    fc = v.pos + glm::dvec3(0, 0, halfSize);
    dist = glm::distance(fc, intersectionPoint);
    if(dist < minDist){
        minDist = dist;
        normal = glm::dvec3(0, 0, 1);
    }
    fc = v.pos + glm::dvec3(0, 0, -halfSize);
    dist = glm::distance(fc, intersectionPoint);
    if(dist < minDist){
        minDist = dist;
        normal = glm::dvec3(0, 0, -1);
    }

    // Compute incidence angle
    double const angle = glm::angle(normal, rayDir);
    return (angle > PI_HALF) ? M_PI - angle : angle;  // Return min. angle
}

std::vector<double> Voxel::getRayIntersection(
    const glm::dvec3& rayOrigin,
    const glm::dvec3& rayDir
){
	return bbox->getRayIntersection(rayOrigin, rayDir);
}
double Voxel::getRayIntersectionDistance(
    const glm::dvec3& rayOrigin,
    const glm::dvec3& rayDir
){
    return bbox->getRayIntersectionDistance(rayOrigin, rayDir);
}

Vertex* Voxel::getVertices() {
	return &v;
}

Vertex* Voxel::getFullVertices(){
    return bbox->getVertices();
}

void Voxel::update() {
	glm::dvec3 hs = glm::dvec3(halfSize, halfSize, halfSize);
	if(bbox != nullptr) delete bbox;
	bbox = new AABB(v.pos - hs, v.pos + hs);
}
