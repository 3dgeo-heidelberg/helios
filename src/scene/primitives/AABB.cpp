#include "AABB.h"
#include "logging.hpp"

#include <glm/gtx/string_cast.hpp>

using namespace std;
using namespace glm;
// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
AABB::AABB(dvec3 min, dvec3 max) {
    this->vertices[0].pos = min;
    this->vertices[1].pos = max;
    this->bounds[0] = min;
    this->bounds[1] = max;
}

Primitive* AABB::clone(){
    AABB *aabb = new AABB();
    _clone(aabb);
    return aabb;
}

void AABB::_clone(Primitive *p){
    Primitive::_clone(p);
    AABB *aabb = (AABB *) p;
    aabb->vertices[0] = Vertex(vertices[0]);
    aabb->vertices[1] = Vertex(vertices[1]);
    aabb->bounds[0] = this->bounds[0];
    aabb->bounds[1] = this->bounds[1];
}


// ***  M E T H O D S  *** //
// *********************** //
dvec3 AABB::getCentroid() {
	dvec3 size = getSize();
	return getMin() + size * 0.5;
}

dvec3 AABB::getSize() {
	return getMax() - getMin();
}

shared_ptr<AABB> AABB::getForPrimitives(std::vector<Primitive*> & primitives) {
	double minX = numeric_limits<double>::max();
	double minY = numeric_limits<double>::max();
	double minZ = numeric_limits<double>::max();

	double maxX = numeric_limits<double>::lowest();
	double maxY = numeric_limits<double>::lowest();
	double maxZ = numeric_limits<double>::lowest();

	for (Primitive *p : primitives) {

		AABB* box = p->getAABB();

		// Find minimum:
		if (box->getMin().x < minX) {
			minX = box->getMin().x;
		}

		if (box->getMin().y < minY) {
			minY = box->getMin().y;
		}

		if (box->getMin().z < minZ) {
			minZ = box->getMin().z;
		}

		// Find maximum:
		if (box->getMax().x > maxX) {
			maxX = box->getMax().x;
		}

		if (box->getMax().y > maxY) {
			maxY = box->getMax().y;
		}

		if (box->getMax().z > maxZ) {
			maxZ = box->getMax().z;
		}
	}

	dvec3 min = dvec3(minX, minY, minZ);
	dvec3 max = dvec3(maxX, maxY, maxZ);

	return shared_ptr<AABB>(new AABB(min, max));
}

shared_ptr<AABB> AABB::getForVertices(vector<Vertex> & verts) {
	double minX = numeric_limits<double>::max();
	double minY = numeric_limits<double>::max();
	double minZ = numeric_limits<double>::max();

	double maxX = numeric_limits<double>::lowest();
	double maxY = numeric_limits<double>::lowest();
	double maxZ = numeric_limits<double>::lowest();

	for (Vertex const & v : verts) {
		// Find minimum:
		if (v.pos.x < minX) {
			minX = v.pos.x;
		}
		if (v.pos.y < minY) {
			minY = v.pos.y;
		}
		if (v.pos.z < minZ) {
			minZ = v.pos.z;
		}
		// Find maximum:
		if (v.pos.x > maxX) {
			maxX = v.pos.x;
		}
		if (v.pos.y > maxY) {
			maxY = v.pos.y;
		}
		if (v.pos.z > maxZ) {
			maxZ = v.pos.z;
		}
	}

	dvec3 min = dvec3(minX, minY, minZ);
	dvec3 max = dvec3(maxX, maxY, maxZ);

	return shared_ptr<AABB>(new AABB(min, max));
}

std::shared_ptr<AABB> AABB::getForVertices(
    std::unordered_set<Vertex *, VertexKeyHash, VertexKeyEqual> & verts
){
    double minX = numeric_limits<double>::max();
    double minY = numeric_limits<double>::max();
    double minZ = numeric_limits<double>::max();

    double maxX = numeric_limits<double>::lowest();
    double maxY = numeric_limits<double>::lowest();
    double maxZ = numeric_limits<double>::lowest();

    for (Vertex * const & v : verts) {
        // Find minimum:
        if (v->pos.x < minX) {
            minX = v->pos.x;
        }
        if (v->pos.y < minY) {
            minY = v->pos.y;
        }
        if (v->pos.z < minZ) {
            minZ = v->pos.z;
        }
        // Find maximum:
        if (v->pos.x > maxX) {
            maxX = v->pos.x;
        }
        if (v->pos.y > maxY) {
            maxY = v->pos.y;
        }
        if (v->pos.z > maxZ) {
            maxZ = v->pos.z;
        }
    }

    dvec3 min = dvec3(minX, minY, minZ);
    dvec3 max = dvec3(maxX, maxY, maxZ);

    return shared_ptr<AABB>(new AABB(min, max));
}

string AABB::toString() {
	return "Min: " + glm::to_string(getMin()) + ", Max: " + glm::to_string(getMax());
}

std::vector<double> AABB::getRayIntersection(const dvec3& orig, const dvec3& dir) {
	// See http://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
    const int xsign = dir.x < 0;
    const int ysign = dir.y < 0;
    const int zsign = dir.z < 0;

	double tmin, tmax;
	tmin = (bounds[xsign].x - orig.x) / dir.x;
	tmax = (bounds[1 - xsign].x - orig.x) / dir.x;
	const double tymin = (bounds[ysign].y - orig.y) / dir.y;
	const double tymax = (bounds[1 - ysign].y - orig.y) / dir.y;

	if (tmin > tymax || tymin > tmax) return std::vector<double>();
	if (tymin > tmin)tmin = tymin;
	if (tymax < tmax) tmax = tymax;

	const double tzmin = (bounds[zsign].z - orig.z) / dir.z;
	const double tzmax = (bounds[1 - zsign].z - orig.z) / dir.z;

	if (tmin > tzmax || tzmin > tmax) return{};

	if (tzmin > tmin) tmin = tzmin;
	if (tzmax < tmax) tmax = tzmax;

	return std::vector<double> { tmin, tmax };
}
double AABB::getRayIntersectionDistance(
    const glm::dvec3& rayOrigin,
    const glm::dvec3& rayDir
){
    const int xsign = rayDir.x < 0;
    const int ysign = rayDir.y < 0;
    const int zsign = rayDir.z < 0;

    double tmin, tmax;
    tmin = (bounds[xsign].x - rayOrigin.x) / rayDir.x;
    tmax = (bounds[1 - xsign].x - rayOrigin.x) / rayDir.x;
    const double tymin = (bounds[ysign].y - rayOrigin.y) / rayDir.y;
    const double tymax = (bounds[1 - ysign].y - rayOrigin.y) / rayDir.y;

    if (tmin > tymax || tymin > tmax) return -1.0;
    if (tymin > tmin)tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    const double tzmin = (bounds[zsign].z - rayOrigin.z) / rayDir.z;
    const double tzmax = (bounds[1 - zsign].z - rayOrigin.z) / rayDir.z;

    if (tmin > tzmax || tzmin > tmax) return -1.0;

    if (tzmin > tmin) tmin = tzmin;

    return tmin;
}

AABB* AABB::getAABB() {
	return this;
}

Vertex* AABB::getVertices() {
	return vertices;
}

double AABB::getIncidenceAngle_rad(
    const glm::dvec3& rayOrigin,
    const glm::dvec3& rayDir,
    const glm::dvec3& intersectionPoint
){
	// TODO 5: Implement this
	return 0;
}
