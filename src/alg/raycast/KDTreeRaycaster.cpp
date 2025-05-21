#include "KDTreeRaycaster.h"
#include "logging.hpp"

using namespace std;

map<double, Primitive*>
KDTreeRaycaster::searchAll(glm::dvec3 const rayOrigin,
                           glm::dvec3 const rayDir,
                           double const tmin,
                           double const tmax,
                           bool const groundOnly)
{
  // Prepare search
  KDTreeRaycasterSearch search(rayDir, rayOrigin, groundOnly);

  // Do recursive search
  this->searchAll_recursive(this->root.get(), tmin, tmax, search);

  // Handle search output
  return search.collectedPoints;
}

RaySceneIntersection*
KDTreeRaycaster::search(glm::dvec3 const rayOrigin,
                        glm::dvec3 const rayDir,
                        double const tmin,
                        double const tmax,
                        bool const groundOnly)
{
  // Prepare search
  KDTreeRaycasterSearch search(rayDir, rayOrigin, groundOnly);

  // Do recursive search
  Primitive* prim = this->search_recursive(
    this->root.get(), tmin - epsilon, tmax + epsilon, search);

  // Handle search output
  if (prim == nullptr)
    return nullptr;
  RaySceneIntersection* result = new RaySceneIntersection();
  result->prim = prim;
  result->point = rayOrigin + (rayDir * search.closestHitDistance);
  result->hitDistance = search.closestHitDistance;
  return result;
}

void
KDTreeRaycaster::searchAll_recursive(LightKDTreeNode* node,
                                     double const tmin,
                                     double const tmax,
                                     KDTreeRaycasterSearch& search)
{

  // ######### BEGIN If node is a leaf, perform ray-primitive intersection on
  // all primitives in the leaf's bucket ###########
  if (node->splitAxis == -1) {
    for (auto prim : *node->primitives) {
      vector<double> tMinMax =
        prim->getRayIntersection(search.rayOrigin, search.rayDir);
      if (tMinMax.empty()) {
        logging::DEBUG("searchAll_recursive: tMinMax is empty");
        continue;
      }
      double newDistance = tMinMax[0];

      // NOTE:
      // Checking for tmin <= newDistance <= tmax here is REQUIRED to prevent
      // the following scenario from producing wrong results:

      // Imagine a primitive extending across multiple partitions (i.e. kdtree
      // leaves). Now, if the tree traversal algorithm arrives at a leaf and
      // checks for ray-primitive-intersections *without* the range check, it
      // might detect an intersection with a primitive that intersects with the
      // partition, but the ray-primitive intersection is *outside* of the
      // partition. Traversal would stop and the intersection would be returned
      // without checking if there are *other* intersections (in other leaves,
      // with other primitives) that are *closer* to the ray originWaypoint. If
      // this was the case, the returned intersection would be wrong.

      if (newDistance > 0 &&
          (newDistance >= tmin - epsilon && newDistance <= tmax + epsilon)) {
        if (!search.groundOnly ||
            (prim->material != nullptr && prim->material->isGround)) {
          search.collectedPoints.insert(
            pair<double, Primitive*>(newDistance, prim));
        }
      }
    }
  }
  // ######### END If node is a leaf, perform ray-primitive intersection on all
  // primitives in the leaf's bucket ###########

  // ######### BEGIN If node is not a leaf, figure out which child node(s) to
  // traverse next, in which order #############
  else {
    int a = node->splitAxis;
    double thit = numeric_limits<double>::infinity();
    LightKDTreeNode* first = nullptr;
    LightKDTreeNode* second = nullptr;

    // ############ BEGIN Check ray direction to figure out through which sides
    // the ray passes in which order ###########

    // Case 1: Ray goes in positive direction - it passes through the left side
    // first, then through the right:
    if (search.rayDir[a] > 0) {
      first = node->left;
      second = node->right;

      thit = (node->splitPos - search.rayOrigin[a]) / search.rayDir[a];
    }
    // Case 2: Ray goes in negative direction - it passes through the right side
    // first, then through the left:
    else if (search.rayDir[a] < 0) {
      first = node->right;
      second = node->left;

      thit = (node->splitPos - search.rayOrigin[a]) / search.rayDir[a];
    }
    // Case 3: Ray goes parallel to the split plane - it passes through only one
    // side, depending on it's originWaypoint:
    else {
      first = (search.rayOrigin[a] < node->splitPos) ? node->left : node->right;
      second =
        (search.rayOrigin[a] < node->splitPos) ? node->right : node->left;
    }
    // ############ END Check ray direction to figure out thorugh which sides
    // the ray passes in which order ###########

    // ########### BEGIN Check where the ray crosses the split plane to figure
    // out which sides we need to stop into at all ###########

    // thit >= tmax means that the ray crosses the split plane *after it has
    // already left the volume*. In this case, it never enters the second half.
    if (thit >= tmax) {
      searchAll_recursive(first, tmin, tmax, search);
    }

    // thit <= tmin means that the ray crosses the split plane *before it enters
    // the volume*. In this case, it never enters the first half:
    else if (thit <= tmin) {
      searchAll_recursive(second, tmin, tmax, search);
    }

    // Otherwise, the ray crosses the split plane within the volume.
    // This means that it passes through both sides:
    else {
      searchAll_recursive(first, tmin, thit, search);
      searchAll_recursive(second, thit, tmax, search);
    }
    // ########### END Check where the ray crosses the split plane to figure out
    // which sides we need to stop into at all ###########
  }
  // ######### END If node is not a leaf, figure out which child node(s) to
  // traverse next, in which order #############
}

Primitive*
KDTreeRaycaster::search_recursive(LightKDTreeNode* node,
                                  double const tmin,
                                  double const tmax,
                                  KDTreeRaycasterSearch& search) const
{
  if (node == nullptr)
    return nullptr; // Null nodes cannot contain primitives
  Primitive* hitPrim = nullptr;

  // ######### BEGIN If node is a leaf, perform ray-primitive intersection on
  // all primitives in the leaf's bucket ###########
  if (node->splitAxis == -1) {
    for (auto prim : *node->primitives) {
      double const newDistance =
        prim->getRayIntersectionDistance(search.rayOrigin, search.rayDir);

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
      if (newDistance > 0 && newDistance < search.closestHitDistance &&
          newDistance >= tmin && newDistance <= tmax) {
        if (!search.groundOnly ||
            (prim->material != nullptr && prim->material->isGround)) {
          search.closestHitDistance = newDistance;
          hitPrim = prim;
        }
      }
    }
  }
  // ######### END If node is a leaf, perform ray-primitive intersection on all
  // primitives in the leaf's bucket ###########

  // ######### BEGIN If node is not a leaf, figure out which child node(s) to
  // traverse next, in which order #############
  else {

    int const a = node->splitAxis;

    double thit = numeric_limits<double>::infinity();

    LightKDTreeNode* first = nullptr;
    LightKDTreeNode* second = nullptr;

    // ############ BEGIN Check ray direction to figure out thorugh which sides
    // the ray passes in which order ###########

    // Case 1: Ray goes in positive direction - it passes through the left side
    // first, then through the right:
    if (search.rayDir[a] > 0) {
      first = node->left;
      second = node->right;

      thit = (node->splitPos - search.rayOrigin[a]) / search.rayDir[a];
    }
    // Case 2: Ray goes in negative direction - it passes through the right side
    // first, then through the left:
    else if (search.rayDir[a] < 0) {
      first = node->right;
      second = node->left;

      thit = (node->splitPos - search.rayOrigin[a]) / search.rayDir[a];
    }
    // Case 3: Ray goes parallel to the split plane - it passes through only one
    // side, depending on it's originWaypoint:
    else {
      first = (search.rayOrigin[a] < node->splitPos) ? node->left : node->right;
      second =
        (search.rayOrigin[a] < node->splitPos) ? node->right : node->left;
    }
    // ############ END Check ray direction to figure out thorugh which sides
    // the ray passes in which order ###########

    // ########### BEGIN Check where the ray crosses the split plane to figure
    // out which sides we need to stop into at all ###########

    // thit >= tmax means that the ray crosses the split plane *after it has
    // already left the volume*. In this case, it never enters the second half.
    if (thit >= tmax) {
      hitPrim = search_recursive(first, tmin, tmax, search);
    }

    // thit <= tmin means that the ray crosses the split plane *before it enters
    // the volume*. In this case, it never enters the first half:
    else if (thit <= tmin) {
      hitPrim = search_recursive(second, tmin, tmax, search);
    }

    // Otherwise, the ray crosses the split plane within the volume.
    // This means that it passes through both sides:
    else {
      hitPrim = search_recursive(first, tmin, thit + epsilon, search);

      if (hitPrim == nullptr) {
        hitPrim = search_recursive(second, thit - epsilon, tmax, search);
      }
    }

    // ########### END Check where the ray crosses the split plane to figure out
    // which sides we need to stop into at all ###########
  }
  // ######### END If node is not a leaf, figure out which child node(s) to
  // traverse next, in which order #############

  return hitPrim;
}
