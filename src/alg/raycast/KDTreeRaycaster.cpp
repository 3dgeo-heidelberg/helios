#include "KDTreeRaycaster.h"
#include "logging.hpp"

using namespace std;

map<double, Primitive*> KDTreeRaycaster::searchAll(
    glm::dvec3 _rayOrigin,
    glm::dvec3 _rayDir,
    double tmin,
    double tmax,
    bool groundOnly
) {
	rayDirArray.push_back(_rayDir.x);
	rayDirArray.push_back(_rayDir.y);
	rayDirArray.push_back(_rayDir.z);
	rayOriginArray.push_back(_rayOrigin.x);
	rayOriginArray.push_back(_rayOrigin.y);
	rayOriginArray.push_back(_rayOrigin.z);

	this->rayOrigin = _rayOrigin;
	this->rayDir = _rayDir;

	this->groundOnly = groundOnly;

	this->collectedPoints.clear();
	this->closestHitDistance = numeric_limits<double>::max();

	this->searchAll_recursive(this->root.get(), tmin, tmax);

	return collectedPoints;
}

RaySceneIntersection* KDTreeRaycaster::search(
    glm::dvec3 _rayOrigin,
    glm::dvec3 _rayDir,
    double tmin,
    double tmax,
    bool groundOnly
){
	rayDirArray.push_back(_rayDir.x);
	rayDirArray.push_back(_rayDir.y);
	rayDirArray.push_back(_rayDir.z);
	rayOriginArray.push_back(_rayOrigin.x);
	rayOriginArray.push_back(_rayOrigin.y);
	rayOriginArray.push_back(_rayOrigin.z);

	this->rayOrigin = _rayOrigin;
	this->rayDir = _rayDir;

	this->groundOnly = groundOnly;

	this->closestHitDistance = numeric_limits<double>::max();

	Primitive* prim = this->search_recursive(
	    this->root.get(),
	    tmin-epsilon,
	    tmax+epsilon
    );
	if (prim == nullptr) return nullptr;

	RaySceneIntersection* result = new RaySceneIntersection();
	result->prim = prim;
	result->point = rayOrigin + (rayDir * closestHitDistance);

	return result;

}

void KDTreeRaycaster::searchAll_recursive(
    LightKDTreeNode* node,
    double tmin,
    double tmax
){

	// ######### BEGIN If node is a leaf, perform ray-primitive intersection on all primitives in the leaf's bucket ###########
	if (node->splitAxis == -1) {
		for (auto prim : *node->primitives) {

			vector<double> tMinMax = prim->getRayIntersection(rayOrigin, rayDir);
			if (tMinMax.empty()) {
				logging::DEBUG("searchAll_recursive: tMinMax is empty");
				continue;
			}
			double newDistance = tMinMax[0];

			// NOTE:
			// Checking for tmin <= newDistance <= tmax here is REQUIRED to prevent the following scenario from producing wrong results:

			// Imagine a primitive extending across multiple partitions (i.e. kdtree leaves). Now, if the tree traversal algorithm
			// arrives at a leaf and checks for ray-primitive-intersections *without* the range check, it might detect an
			// intersection with a primitive that intersects with the partition, but the ray-primitive intersection is *outside*
			// of the partition. Traversal would stop and the intersection would be returned without checking if there are
			// *other* intersections (in other leaves, with other primitives) that are *closer* to the ray originWaypoint. If this was
			// the case, the returned intersection would be wrong.

			if(
			    newDistance > 0 &&
			    (newDistance >= tmin-epsilon && newDistance <= tmax+epsilon)
            ){
				if(
				    !groundOnly ||
				    (prim->material != nullptr && prim->material->isGround)
                ){
					collectedPoints.insert(pair<double, Primitive*>(
					    newDistance,
					    prim)
                    );
				}
			}
		}
	}
	// ######### END If node is a leaf, perform ray-primitive intersection on all primitives in the leaf's bucket ###########

	// ######### BEGIN If node is not a leaf, figure out which child node(s) to traverse next, in which order #############
	else {
		int a = node->splitAxis;
		double thit = numeric_limits<double>::infinity();
		LightKDTreeNode* first = nullptr;
		LightKDTreeNode* second = nullptr;

		// ############ BEGIN Check ray direction to figure out through which sides the ray passes in which order ###########

		// Case 1: Ray goes in positive direction - it passes through the left side first, then through the right:
		if (rayDirArray[a] > 0) {
			first = node->left;
			second = node->right;

			thit = (node->splitPos - this->rayOriginArray[a]) / rayDirArray[a];
		}
		// Case 2: Ray goes in negative direction - it passes through the right side first, then through the left:
		else if (rayDirArray[a] < 0) {
			first = node->right;
			second = node->left;

			thit = (node->splitPos - this->rayOriginArray[a]) / rayDirArray[a];
		}
		// Case 3: Ray goes parallel to the split plane - it passes through only one side, depending on it's originWaypoint:
		else {
			first = (rayOriginArray[a] < node->splitPos) ? node->left : node->right;
			second = (rayOriginArray[a] < node->splitPos) ? node->right : node->left;
		}
		// ############ END Check ray direction to figure out thorugh which sides the ray passes in which order ###########

		// ########### BEGIN Check where the ray crosses the split plane to figure out which sides we need to stop into at all ###########

		// thit >= tmax means that the ray crosses the split plane *after it has already left the volume*.
		// In this case, it never enters the second half.
		if (thit >= tmax) {
			searchAll_recursive(first, tmin, tmax);
		}

		// thit <= tmin means that the ray crosses the split plane *before it enters the volume*.
		// In this case, it never enters the first half:
		else if (thit <= tmin) {
			searchAll_recursive(second, tmin, tmax);
		}

		// Otherwise, the ray crosses the split plane within the volume.
		// This means that it passes through both sides:
		else {
			searchAll_recursive(first, tmin, thit);
			searchAll_recursive(second, thit, tmax);
		}
		// ########### END Check where the ray crosses the split plane to figure out which sides we need to stop into at all ###########
	}
	// ######### END If node is not a leaf, figure out which child node(s) to traverse next, in which order #############
}

Primitive* KDTreeRaycaster::search_recursive(
    LightKDTreeNode* node,
    double tmin,
    double tmax
){
    if(node==nullptr) return nullptr; // Null nodes cannot contain primitives
	Primitive* hitPrim = nullptr;

	// ######### BEGIN If node is a leaf, perform ray-primitive intersection on all primitives in the leaf's bucket ###########
	if (node->splitAxis == -1) {
		//logging::DEBUG("leaf node:");
		for (auto prim : *node->primitives) {
			double newDistance =
			    prim->getRayIntersectionDistance(rayOrigin, rayDir);

			// NOTE:
			// Checking for tmin <= newDistance <= tmax here is REQUIRED
			// to prevent the following scenario from producing wrong results:

			// Imagine a primitive extending across multiple partitions
			// (i.e. kdtree leaves).
			// Now, if the tree traversal algorithm
			// arrives at a leaf and checks for ray-primitive-intersections
			// *without* the range check, it might detect an intersection
			// with a primitive that intersects with the partition, but
			// the ray-primitive intersection is *outside* of the partition.
			// Traversal would stop and the intersection would be returned
			// without checking if there are *other* intersections
			// (in other leaves, with other primitives) that are *closer* to
			// the ray originWaypoint. If this was the case, the returned intersection
			// would be wrong.

			if(
			    (newDistance > 0 && newDistance < closestHitDistance) &&
			    (newDistance >= tmin && newDistance <= tmax)
            ){
				if(
				    !groundOnly ||
				    (prim->material != nullptr && prim->material->isGround)
                ){
					closestHitDistance = newDistance;
					hitPrim = prim;
				}
			}
		}
	}
	// ######### END If node is a leaf, perform ray-primitive intersection on all primitives in the leaf's bucket ###########

	// ######### BEGIN If node is not a leaf, figure out which child node(s) to traverse next, in which order #############
	else {

		int a = node->splitAxis;

		double thit = numeric_limits<double>::infinity();

		LightKDTreeNode* first = nullptr;
		LightKDTreeNode* second = nullptr;

		// ############ BEGIN Check ray direction to figure out thorugh which sides the ray passes in which order ###########

		// Case 1: Ray goes in positive direction - it passes through the left side first, then through the right:
		if (rayDirArray[a] > 0) {
			first = node->left;
			second = node->right;

			thit = (node->splitPos - this->rayOriginArray[a]) / rayDirArray[a];
		}
		// Case 2: Ray goes in negative direction - it passes through the right side first, then through the left:
		else if (rayDirArray[a] < 0) {
			first = node->right;
			second = node->left;

			thit = (node->splitPos - this->rayOriginArray[a]) / rayDirArray[a];
		}
		// Case 3: Ray goes parallel to the split plane - it passes through only one side, depending on it's originWaypoint:
		else {
			first = (rayOriginArray[a] < node->splitPos) ?
			    node->left : node->right;
			second = (rayOriginArray[a] < node->splitPos) ?
			    node->right : node->left;
		}
		// ############ END Check ray direction to figure out thorugh which sides the ray passes in which order ###########

		// ########### BEGIN Check where the ray crosses the split plane to figure out which sides we need to stop into at all ###########

		// thit >= tmax means that the ray crosses the split plane *after it has already left the volume*.
		// In this case, it never enters the second half.
		if (thit >= tmax) {
			hitPrim = search_recursive(first, tmin, tmax);
		}

		// thit <= tmin means that the ray crosses the split plane *before it enters the volume*.
		// In this case, it never enters the first half:
		else if (thit <= tmin) {
			hitPrim = search_recursive(second, tmin, tmax);
		}

		// Otherwise, the ray crosses the split plane within the volume.
		// This means that it passes through both sides:
		else {
			hitPrim = search_recursive(first, tmin, thit+epsilon);

			if (hitPrim == nullptr) {
				hitPrim = search_recursive(second, thit-epsilon, tmax);
			}
		}

		// ########### END Check where the ray crosses the split plane to figure out which sides we need to stop into at all ###########
	}
	// ######### END If node is not a leaf, figure out which child node(s) to traverse next, in which order #############

	return hitPrim;
}
