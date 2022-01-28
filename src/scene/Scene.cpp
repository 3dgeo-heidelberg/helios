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
#include <surfaceinspector/maths/Plane.hpp>
#include <surfaceinspector/maths/PlaneFitter.hpp>

using SurfaceInspector::maths::Plane;
using SurfaceInspector::maths::PlaneFitter;

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
      kdtf->makeFromPrimitivesUnsafe(this->primitives)
  );
  registerParts();
}

// ***  M E T H O D S  *** //
// *********************** //
bool Scene::finalizeLoading(bool const safe) {
  if (primitives.empty()) return false;

  // #####   UPDATE PRIMITIVES ON FINISH LOADING   #####
  UniformNoiseSource<double> uns(*DEFAULT_RG, -1, 1);
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

  if (vertices.size() == 0) return false;

  // Register all parts and translate to ground those flagged as forceOnGround
  registerParts();
  doForceOnGround();

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

  // Compute each part centroid wrt to scene
  for(shared_ptr<ScenePart> & part : parts) part->computeCentroid();

  // ############# BEGIN Build KD-tree ##################
  if(kdtf != nullptr) buildKDTreeWithLog(safe);
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

  // TODO Rethink : Is it effecient to instantiate raycaster for each check ?
  KDTreeRaycaster raycaster(kdtree);
  return raycaster.searchAll(
      rayOrigin, rayDir, tMinMax[0], tMinMax[1], groundOnly
  );
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

void Scene::doForceOnGround(){

    // 1. Find min and max vertices of ground scene parts
    vector<size_t> I; // Indices of ground parts
    vector<unique_ptr<Plane<double>>> planes; // Ground best fitting planes
    size_t const m = parts.size(); // How many parts there are in the scene
    for(size_t i=0 ; i < m ; ++i){
       shared_ptr<ScenePart> part = parts[i];
       if(!part->mPrimitives[0]->material->isGround) continue;
       I.push_back(i); // Store index of found ground part
       planes.push_back(nullptr); // Null placeholder for best fitting plane
       part->computeCentroid(true); // True implies also store boundaries
    }
    if(I.empty()){ // Check there is at least one ground part
        std::stringstream ss;
        ss  << "Scene::doForceGround could not compute nothing because there "
            << "was no ground scene part available";
        logging::WARN(ss.str());
    }

    // Compute remaining algorithm steps for each on ground scene part
    for(shared_ptr<ScenePart> & part : parts){
        if(part->forceOnGround == 0){ // Ignore not on ground scene parts
            std::stringstream ss;
            ss  << "Scene::doForceOnGround skipped part \""
                << part->mId << "\"\n"
                << "Its forceOnGround attribute was 0";
            logging::DEBUG(ss.str());
            continue;
        }
        else{ // Report search depth (forceOnGround) as debug info
            std::stringstream ss;
            ss  << "Scene::doForceOnGround computing part \""
                << part->mId << "\"\n"
                << "Search depth is " << part->forceOnGround;
            logging::DEBUG(ss.str());
        }
        // 2. Find minimum z vertex and pick first ground reference
        vector<Vertex *> vertices = part->getAllVertices();
        glm::dvec3 minzv = vertices[0]->pos; // First vertex as minz candidate
        size_t const n = vertices.size();
        for(size_t i = 1 ; i < n ; ++i){ // Find best minz candidate
            Vertex *vertex = vertices[i];
            if(vertex->pos.z < minzv.z) minzv = vertex->pos;
        }
        size_t groundLocalIndex;
        shared_ptr<ScenePart> groundPart = nullptr; // Ground scene part
        for(size_t j = 0 ; j < I.size() ; ++j){
            size_t const i = I[j]; // Ground index i
            shared_ptr<ScenePart> groundCandidate = parts[i];
            if( // Ground candidate is valid if minz vertex lies inside in R2
                minzv.x >= groundCandidate->bound->getMin().x &&
                minzv.x <= groundCandidate->bound->getMax().x &&
                minzv.y >= groundCandidate->bound->getMin().y &&
                minzv.y <= groundCandidate->bound->getMax().y
            ){
                groundPart = groundCandidate;
                groundLocalIndex = j;
                break;
            }
        }
        if(groundPart == nullptr){
            std::stringstream ss;
            ss  << "Scene::doForceOnGround could not place part \""
                << part->mId << "\" on ground.\n"
                << "No valid ground candidate was found";
            logging::WARN(ss.str());
            continue;
        }
        // 3. Find ground reference best fitting plane
        if(planes[groundLocalIndex] == nullptr){ // Estimate plane if needed
            vector<Vertex *> groundVertices = groundPart->getAllVertices();
            size_t const ngv = groundVertices.size();
            size_t const ngv2 = 2*ngv;
            arma::mat groundVerticesMatrix(ngv, 3);
            for(size_t i = 0 ; i < ngv ; ++i){
                glm::dvec3 const &vert = groundVertices[i]->pos;
                groundVerticesMatrix[i] = vert.x;
                groundVerticesMatrix[ngv+i] = vert.y;
                groundVerticesMatrix[ngv2+i] = vert.z;
            }
            planes[groundLocalIndex] = unique_ptr<Plane<double>>(
                new Plane<double>(
                    PlaneFitter::bestFittingPlaneSVD<double>(
                        groundVerticesMatrix
                    )
                )
            );
        }
        // 4. Compute the vertical projection of min vertex on ground plane
        vector<double> const &o = planes[groundLocalIndex]->centroid;
        vector<double> const &v = planes[groundLocalIndex]->orthonormal;
        glm::dvec3 q = findForceOnGroundQ(
            part->forceOnGround,
            minzv,
            vertices,
            o,
            v
        );
        double zDelta = q.z -
            (v[0]*o[0]+v[1]*o[1]+v[2]*o[2]-v[0]*q.x-v[1]*q.y) / v[2];
        // 5. Do vertical translation for all vertex of onGround part
        for(Vertex *vertex : vertices) vertex->pos.z -= zDelta;
    }
}


glm::dvec3 Scene::findForceOnGroundQ(
    int const searchDepth,
    glm::dvec3 const minzv,
    vector<Vertex *> &vertices,
    vector<double> const &o,
    vector<double> const &v
){
    if(searchDepth == -1 || searchDepth > 1){ // Iterative search process for q
        // Compute loop configuration
        size_t const maxIters = (searchDepth==-1) ?
            vertices.size() : std::min<size_t>(searchDepth, vertices.size());
        size_t const stepSize = (searchDepth==-1) ?
            1 : (size_t) (vertices.size()/maxIters);
        // Compute the iterative search itself : argmin zDelta
        double const dot = v[0]*o[0]+v[1]*o[1]+v[2]*o[2];
        glm::dvec3 qBest = vertices[0]->pos;
        double zDeltaBest = qBest.z - (dot-v[0]*qBest.x-v[1]*qBest.y)/v[2];
        for(size_t i = stepSize ; i < maxIters ; i+=stepSize){
            glm::dvec3 const q  = vertices[i]->pos;
            double const zDelta = q.z - (dot-v[0]*q.x-v[1]*q.y)/v[2];
            if(zDelta < zDeltaBest){
                zDeltaBest = zDelta;
                qBest = q;
            }
        }
        return qBest;
    }
    // By default searchDepth 1 is assumed, so q=q_*
    return minzv;
}

void Scene::buildKDTree(bool const safe){
    kdtree = shared_ptr<KDTreeNodeRoot>(
        safe ?
        kdtf->makeFromPrimitives(primitives, true, true) :
        kdtf->makeFromPrimitivesUnsafe(primitives, true, true)
    );
}

void Scene::buildKDTreeWithLog(bool const safe){
    logging::INFO("Building KD-Tree... ");
    TimeWatcher kdtTw;
    kdtTw.start();
    buildKDTree(safe);
    kdtTw.stop();
    std::stringstream ss;
    ss << "KDT built in " << kdtTw.getElapsedDecimalSeconds() << "s";
    logging::TIME(ss.str());
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
