//#include <iostream>
#include "logging.hpp"

#include <fstream>
//#include <set>
#include <unordered_set>

#include <SerialIO.h>

#include <chrono>
using namespace std::chrono;
using namespace std;

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
using namespace glm;

#include "KDTreeRaycaster.h"

#include "Scene.h"
#include "TimeWatcher.h"
#include <UniformNoiseSource.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Scene::Scene(Scene &s) {
  if (s.bbox == nullptr) this->bbox = nullptr;
  else this->bbox = std::shared_ptr<AABB>((AABB *)s.bbox->clone());
  if (s.bbox_crs == nullptr) this->bbox_crs = nullptr;
  else this->bbox_crs = std::shared_ptr<AABB>((AABB *)s.bbox_crs->clone());
  std::set<ScenePart *> parts; // Pointer to each ScenePart, no repeats
  std::vector<Primitive *> nonPartPrimitives; // Primitives without ScenePart
  ScenePart *_sp;
  Primitive *_p;
  for (size_t i = 0; i < s.primitives.size(); i++) { // Fill parts
    _p = s.primitives[i];
    _sp = _p->part.get();
    if (_sp != nullptr)
      parts.insert(_sp); // Primitive with part
    else
      nonPartPrimitives.push_back(_p); // Primitive with no part
  }
  for (ScenePart *sp : parts) { // Handle primitives associated with ScenePart
    std::shared_ptr<ScenePart> spc = std::make_shared<ScenePart>(*sp);
    for (Primitive *p : spc->mPrimitives) {
      Primitive *_p = p;
      _p->part = spc;
      this->primitives.push_back(_p);
    }
  }
  for (Primitive *p : nonPartPrimitives) { // Handle primitives with no part
    this->primitives.push_back(p->clone());
  }

  this->kdtree = shared_ptr<KDTreeNodeRoot>(KDTreeNodeRoot::build(primitives));
}

// ***  M E T H O D S  *** //
// *********************** //
bool Scene::finalizeLoading() {
  if (primitives.size() == 0) {
    return false;
  }

  // #####   UPDATE PRIMITIVES ON FINISH LOADING   #####
  UniformNoiseSource<double> uns(-1, 1);
  for (Primitive *p : primitives) {
    p->onFinishLoading(uns);
  }

  // ################ BEGIN Shift primitives to originWaypoint
  // ##################

  // Translate scene coordinates to originWaypoint (to prevent wasting of
  // floating point precision):

  vector<Vertex> vertices;

  // Collect all vertices in an unordered set
  // (makes sure that we don't translate the same vertex multiple times)
  for (Primitive *p : primitives) {
    Vertex *v = p->getFullVertices();
    for (size_t i = 0; i < p->getNumFullVertices(); i++) {
      vertices.push_back(v[i]);
    }
  }
  ostringstream s;
  s << "Total # of primitives in scene: " << primitives.size() << "\n";
  logging::DEBUG(s.str());

  if (vertices.size() == 0) {
    return false;
  }

  // ########## BEGIN Move the scene so that bounding box minimum is (0,0,0)
  // ######## This is done to prevent precision problems (e.g. camera jitter)

  // Store original bounding box (CRS coordinates):
  this->bbox_crs = AABB::getForVertices(vertices);

  dvec3 diff = this->bbox_crs->getMin();

  stringstream ss;
  ss << "CRS bounding box (by vertices): " << this->bbox_crs->toString()
     << "\nShift: " << glm::to_string(diff)
     << "\n# vertices to translate: " << vertices.size();
  logging::INFO(ss.str());
  ss.str("");

  // Iterate over the hash set and translate each vertex:
  for (Vertex &v : vertices) {
    v.pos = v.pos - diff;
  }

  for (Primitive *p : primitives) {
    Vertex *v = p->getVertices();
    for (size_t i = 0; i < p->getNumVertices(); i++) {
      v[i].pos = v[i].pos - diff;
    }
    p->update();
  }

  // Get new bounding box of tranlated scene:
  this->bbox = AABB::getForVertices(vertices);

  ss << "Actual bounding box (by vertices): " << this->bbox->toString();
  logging::INFO(ss.str());
  ss.str("");

  // ################ END Shift primitives to originWaypoint ##################

  // ############# BEGIN Build KD-tree ##################
  logging::INFO("Building KD-Tree... ");

  TimeWatcher tw;
  tw.start();
  kdtree = shared_ptr<KDTreeNodeRoot>(KDTreeNodeRoot::build(primitives));

  tw.stop();
  ss << "KD built in " << tw.getElapsedDecimalSeconds() << "s";
  logging::INFO(ss.str());
  // ############# END Build KD-tree ##################

  return true;
}

shared_ptr<AABB> Scene::getAABB() { return this->bbox; }

dvec3 Scene::getGroundPointAt(dvec3 point) {

  dvec3 origin = dvec3(point.x, point.y, bbox->getMin()[2] - 0.1);
  dvec3 dir = dvec3(0, 0, 1);

  shared_ptr<RaySceneIntersection> intersect =
      getIntersection(origin, dir, true);

  if (intersect == nullptr) {
    stringstream ss;
    ss << "getGroundPointAt(" << point.x << "," << point.y << "," << point.z
       << ") : intersect is NULL";
    ss << "\n\torigin = (" << origin.x << ", " << origin.y << ", " << origin.z
       << ");\n\tdir = (" << dir.x << ", " << dir.y << ", " << dir.z << ");"
       << std::endl;
    logging::DEBUG(ss.str());
    return {};
  }

  intersect->point.z += intersect->prim->getGroundZOffset();
  return intersect->point;
}

shared_ptr<RaySceneIntersection>
Scene::getIntersection(dvec3 &rayOrigin, dvec3 &rayDir, bool groundOnly) {
  vector<double> tMinMax = bbox->getRayIntersection(rayOrigin, rayDir);
  if (tMinMax.empty()) {
    logging::DEBUG("tMinMax is empty");
    return nullptr;
  }

  // TODO test without kdtree
  bool bruteForce = false;
  shared_ptr<RaySceneIntersection> result;

  if (!bruteForce) {
    KDTreeRaycaster raycaster(kdtree);
    result = shared_ptr<RaySceneIntersection>(raycaster.search(
        rayOrigin, rayDir, tMinMax[0], tMinMax[1], groundOnly));
  }

  return result;
}

map<double, Primitive *>
Scene::getIntersections(dvec3 &rayOrigin, dvec3 &rayDir, bool groundOnly) {

  vector<double> tMinMax = bbox->getRayIntersection(rayOrigin, rayDir);
  if (tMinMax.empty()) {
    logging::DEBUG("tMinMax is empty");
    return {};
  }

  shared_ptr<KDTreeRaycaster> raycaster(new KDTreeRaycaster(kdtree));
  return raycaster->searchAll(rayOrigin, rayDir, tMinMax[0], tMinMax[1],
                              groundOnly);
}

dvec3 Scene::getShift() { return this->bbox_crs->getMin(); }

void Scene::writeObject(string path) {
  stringstream ss;
  ss << "Writing " << path << " ...";
  logging::INFO(ss.str());
  SerialIO::getInstance()->write<Scene>(path, this);
}

Scene *Scene::readObject(string path) {
  stringstream ss;
  ss << "Reading scene object " << path << " ...";
  logging::INFO(ss.str());
  return SerialIO::getInstance()->read<Scene>(path);
}