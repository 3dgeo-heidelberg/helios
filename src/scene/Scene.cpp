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
  std::set<ScenePart *> _parts; // Pointer to each ScenePart, no repeats
  std::vector<Primitive *> nonPartPrimitives; // Primitives without ScenePart
  ScenePart *_sp;
  Primitive *_p;
  for (size_t i = 0; i < s.primitives.size(); i++) { // Fill parts
    _p = s.primitives[i];
    _sp = _p->part.get();
    if (_sp != nullptr) _parts.insert(_sp); // Primitive with part
    else nonPartPrimitives.push_back(_p); // Primitive with no part
  }
  for (ScenePart *sp : _parts) { // Handle primitives associated with ScenePart
    std::shared_ptr<ScenePart> spc = std::make_shared<ScenePart>(*sp);
    for (Primitive *p : spc->mPrimitives) {
      p->part = spc;
      this->primitives.push_back(p);
    }
  }
  for (Primitive *p : nonPartPrimitives) { // Handle primitives with no part
    this->primitives.push_back(p->clone());
  }

  this->kdtf = s.kdtf;
  this->kdtree = shared_ptr<KDTreeNodeRoot>(
      kdtf->makeFromPrimitives(this->primitives)
  );
  registerParts();
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

  glm::dvec3 diff = this->bbox_crs->getMin();

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

  // Register parts and compute its centroid wrt to scene
  registerParts();
  for(shared_ptr<ScenePart> & part : parts) part->computeCentroid();

  // ############# BEGIN Build KD-tree ##################
  logging::INFO("Building KD-Tree... ");

  TimeWatcher tw;
  tw.start();
  kdtree = shared_ptr<KDTreeNodeRoot>(kdtf->makeFromPrimitives(primitives));

  tw.stop();
  ss << "KD built in " << tw.getElapsedDecimalSeconds() << "s";
  logging::INFO(ss.str());
  // ############# END Build KD-tree ##################

  return true;
}

void Scene::registerParts(){
    unordered_set<shared_ptr<ScenePart>> partsSet;
    for(Primitive *primitive : primitives)
        if(primitive->part != nullptr)
            partsSet.insert(primitive->part);
    parts = vector<shared_ptr<ScenePart>>(partsSet.begin(), partsSet.end());
}

shared_ptr<AABB> Scene::getAABB() { return this->bbox; }

glm::dvec3 Scene::getGroundPointAt(glm::dvec3 point) {

  glm::dvec3 origin = glm::dvec3(point.x, point.y, bbox->getMin()[2] - 0.1);
  glm::dvec3 dir = glm::dvec3(0, 0, 1);

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
Scene::getIntersection(
    glm::dvec3 &rayOrigin,
    glm::dvec3 &rayDir,
    bool groundOnly
){
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
Scene::getIntersections(
    glm::dvec3 &rayOrigin,
    glm::dvec3 &rayDir,
    bool groundOnly
){

  vector<double> tMinMax = bbox->getRayIntersection(rayOrigin, rayDir);
  if (tMinMax.empty()) {
    logging::DEBUG("tMinMax is empty");
    return {};
  }

  shared_ptr<KDTreeRaycaster> raycaster(new KDTreeRaycaster(kdtree));
  return raycaster->searchAll(rayOrigin, rayDir, tMinMax[0], tMinMax[1],
                              groundOnly);
}

glm::dvec3 Scene::getShift() { return this->bbox_crs->getMin(); }

vector<Vertex *> Scene::getAllVertices(){
    unordered_set<Vertex *> vset;
    for(Primitive *primitive : primitives){
        size_t const m  = primitive->getNumVertices();
        Vertex *vertices = primitive->getVertices();
        for(size_t i = 0 ; i < m ; ++i) vset.insert(vertices + i);
    }
    return {vset.begin(), vset.end()};
}

// ***  READ/WRITE  *** //
// ******************** //
void Scene::writeObject(string path) {
    stringstream ss;
    ss << "Writing scene object to " << path << " ...";
    logging::INFO(ss.str());
    SerialIO::getInstance()->write<Scene>(path, this);
}

Scene *Scene::readObject(string path) {
    stringstream ss;
    ss << "Reading scene object from " << path << " ...";
    logging::INFO(ss.str());
    return SerialIO::getInstance()->read<Scene>(path);
}
