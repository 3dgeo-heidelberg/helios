#include <boost/lexical_cast.hpp>
#include <boost/lexical_cast/bad_lexical_cast.hpp>

#include <ScenePart.h>
#include <Primitive.h>
#include <Triangle.h>
#include <AABB.h>
#include <WavefrontObj.h>
#include <util/logger/logging.hpp>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScenePart::ScenePart(ScenePart const &sp, bool const shallowPrimitives) {
  this->centroid = sp.centroid;
  this->bound = sp.bound;
  this->mId = sp.mId;
  this->onRayIntersectionMode = sp.onRayIntersectionMode;
  this->onRayIntersectionArgument = sp.onRayIntersectionArgument;
  this->randomShift = sp.randomShift;
  this->mOrigin = glm::dvec3(sp.mOrigin);
  this->mRotation = Rotation(sp.mRotation);
  this->mScale = sp.mScale;
  this->forceOnGround = forceOnGround;
  this->mCrs = nullptr; // TODO Copy this too
  this->mEnv = nullptr; // TODO Copy this too

  this->primitiveType = sp.primitiveType;
  this->mPrimitives = std::vector<Primitive *>(0);
  Primitive *p;

  if(shallowPrimitives){
      for (size_t i = 0; i < sp.mPrimitives.size(); ++i) {
          this->mPrimitives.push_back(sp.mPrimitives[i]);
      }
  }
  else{
      for (size_t i = 0; i < sp.mPrimitives.size(); ++i) {
          p = sp.mPrimitives[i]->clone();
          p->part = sp.mPrimitives[i]->part;
          this->mPrimitives.push_back(p);
      }
  }

  this->subpartLimit = sp.subpartLimit;
}

// ***  COPY / MOVE OPERATORS  *** //
// ******************************* //
ScenePart &ScenePart::operator=(const ScenePart &rhs) {
  this->centroid = rhs.centroid;
  this->bound = rhs.bound;
  this->mId = rhs.mId;
  this->onRayIntersectionMode = rhs.onRayIntersectionMode;
  this->onRayIntersectionArgument = rhs.onRayIntersectionArgument;
  this->randomShift = rhs.randomShift;
  this->mOrigin = glm::dvec3(rhs.mOrigin);
  this->mRotation = Rotation(rhs.mRotation);
  this->mScale = rhs.mScale;
  this->forceOnGround = rhs.forceOnGround;
  this->mCrs = nullptr; // TODO Copy this too
  this->mEnv = nullptr; // TODO Copy this too

  this->mPrimitives = std::vector<Primitive *>(0);
  Primitive *p;
  for (size_t i = 0; i < rhs.mPrimitives.size(); i++) {
    p = rhs.mPrimitives[i]->clone();
    p->part = rhs.mPrimitives[i]->part;
    this->mPrimitives.push_back(p);
  }

  this->subpartLimit = rhs.subpartLimit;

  return *this;
}

// ***  M E T H O D S  *** //
// *********************** //

void ScenePart::addObj(WavefrontObj *obj) {
  std::stringstream ss;
  ss << "Adding primitive to Scenepart ...";
  logging::DEBUG(ss.str());
  ss.str("");
  size_t oldNumPrimitives = mPrimitives.size();

  Primitive *p;
  for (size_t i = 0; i < obj->primitives.size(); i++) {
    p = obj->primitives[i]->clone();
    mPrimitives.push_back(p);
  }

  ss << "# new primitives added: " << mPrimitives.size() - oldNumPrimitives;
  logging::DEBUG(ss.str());
}

std::vector<Vertex *> ScenePart::getAllVertices() const {
  std::vector<Vertex *> allPos;
  for (Primitive *p : mPrimitives) {
    for (size_t i = 0; i < p->getNumVertices(); i++) {
      allPos.push_back(p->getVertices() + i);
    }
  }
  return allPos;
}

void ScenePart::smoothVertexNormals() {
  Triangle *t;
  Vertex *v;
  std::map<Vertex *, std::shared_ptr<std::vector<Triangle *>>> vtmap;

  // Build map so for each vertex all triangles formed by it are known
  for (size_t i = 0; i < mPrimitives.size(); i++) {
    t = (Triangle *)mPrimitives[i];
    for (int j = 0; j <= 2; j++) {
      if (!vtmap.count(t->verts + j)) {
        std::shared_ptr<std::vector<Triangle *>> vec =
            std::make_shared<std::vector<Triangle *>>();
        vtmap.insert({t->verts + j, vec});
      }
      vtmap[t->verts + j]->push_back(t);
    }
  }

  // Set the normal of each vertex as the mean of each triangle normal
  std::shared_ptr<std::vector<Triangle *>> vec;
  for (std::map<Vertex *, std::shared_ptr<std::vector<Triangle *>>>::iterator
           iter = vtmap.begin();
       iter != vtmap.end(); iter++) {
    v = iter->first;
    vec = iter->second;
    v->normal[0] = 0.0;
    v->normal[1] = 0.0;
    v->normal[2] = 0.0;
    for (Triangle *t : *vec) {
      v->normal += t->getFaceNormal();
    }
    v->normal = glm::normalize(v->normal);
  }
}

bool ScenePart::splitSubparts() {
  size_t n = subpartLimit.size();
  if (n <= 1)
    return false; // There is no need to do splits

  // Prepare incremental ID
  int start = -1;
  try {
    start = boost::lexical_cast<int>(mId);
  } catch (boost::bad_lexical_cast &blcex) {
    std::stringstream ss;
    ss << "Could not update subpart ID from \"" << mId << "\".\n "
       << "Thus, splitting subparts is aborted";
    logging::WARN(ss.str());
    return false;
  }

  // Do splits
  for (size_t i = 1; i < n; ++i) {
    std::shared_ptr<ScenePart> newPart = std::make_shared<ScenePart>();
    newPart->onRayIntersectionMode = onRayIntersectionMode;
    newPart->onRayIntersectionArgument = onRayIntersectionArgument;
    newPart->randomShift = randomShift;
    if (ladlut == nullptr) newPart->ladlut = nullptr;
    else newPart->ladlut = std::make_shared<LadLut>(*ladlut);
    newPart->mOrigin = mOrigin;
    newPart->mRotation = mRotation;
    newPart->mScale = mScale;
    newPart->forceOnGround = forceOnGround;
    for (size_t j = subpartLimit[i - 1]; j < subpartLimit[i]; ++j) {
      newPart->mPrimitives.push_back(mPrimitives[j]);
      mPrimitives[j]->part = newPart;
    }
    newPart->mId = std::to_string(start + i);
  }

  // Remove splitted primitives
  mPrimitives.erase(mPrimitives.begin() + subpartLimit[0], mPrimitives.end());
  subpartLimit.clear();
  mId = std::to_string(start);

  /*
   * /!\  WARNING  /!\
   * The origin, as rotation and scale transformations, are applied before
   *  splitting. Hence, transformations are applied with respect to the
   *  original scene part.
   * Thus, each new scene part coming from splitting the original one stills
   *  considering the origin and transformation specifications of original
   *  scene part.
   * In consequence, they can be easily reverted. But, to apply
   *  transformations to new scene parts, their attributes such as origin
   *  should be updated to new primitive. Consider this when manipulating
   *  those subparts in future. The original purpose for this split was to
   *  have different hitObjectId for different components.
   */
  return true;
}

void ScenePart::computeCentroid(bool const computeBound){
    // Find centroid coordinates
    double xmin=std::numeric_limits<double>::max();
    double xmax=std::numeric_limits<double>::lowest();
    double ymin=xmin, ymax=xmax, zmin=xmin, zmax=xmax;
    std::vector<Vertex*> vertices = getAllVertices();
    for(Vertex * vertex : vertices){
        // Find centroid x coordinate
        double const x = vertex->getX();
        if(x < xmin) xmin = x;
        if(x > xmax) xmax = x;
        // Find centroid y coordinate
        double const y = vertex->getY();
        if(y < ymin) ymin = y;
        if(y > ymax) ymax = y;
        // Find centroid z coordinate
        double const z = vertex->getZ();
        if(z < zmin) zmin = z;
        if(z > zmax) zmax = z;
    }

    // Build the centroid
    centroid = arma::colvec(std::vector<double>({
        (xmin+xmax)/2.0,
        (ymin+ymax)/2.0,
        (zmin+zmax)/2.0,
    }));

    // Build the bound
    if(computeBound){
        bound = make_shared<AABB>(
            xmin, ymin, zmin, xmax, ymax, zmax
        );
    }
}

void ScenePart::computeTransformations(
    std::shared_ptr<ScenePart> sp,
    bool const holistic
){
    // For all primitives, set reference to their scene part and transform:
    for (Primitive *p : sp->mPrimitives) {
        p->part = sp;
        p->rotate(sp->mRotation);
        if (holistic) {
            for (size_t i = 0; i < p->getNumVertices(); i++) {
                p->getVertices()[i].pos.x *= sp->mScale;
                p->getVertices()[i].pos.y *= sp->mScale;
                p->getVertices()[i].pos.z *= sp->mScale;
            }
        }
        p->scale(sp->mScale);
        p->translate(sp->mOrigin);
    }
}

void ScenePart::release(){
    for(Primitive *p : mPrimitives){
        delete p;
    }
    mPrimitives.clear();
    if(sorh != nullptr){
        for(Primitive * p : sorh->getBaselinePrimitives()){
            delete p;
        }
        sorh->baseline = nullptr;
        sorh = nullptr;
    }
}
