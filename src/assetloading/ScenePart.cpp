#include <boost/lexical_cast/bad_lexical_cast.hpp>
#include <boost/lexical_cast.hpp>

#include <util/logger/logging.hpp>
#include "ScenePart.h"
#include "Triangle.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScenePart::ScenePart(ScenePart &sp){
    this->mId = sp.mId;
    this->onRayIntersectionMode = sp.onRayIntersectionMode;
    this->onRayIntersectionArgument = sp.onRayIntersectionArgument;
    this->randomShift = sp.randomShift;
    this->mOrigin = glm::dvec3(sp.mOrigin);
    this->mRotation = Rotation(sp.mRotation);
    this->mScale = sp.mScale;
    this->mCrs = nullptr; // TODO Copy this too
    this->mEnv = nullptr; // TODO Copy this too

    this->mPrimitives = std::vector<Primitive *>(0);
    Primitive *p;
    std::shared_ptr<ScenePart> thisShared = std::shared_ptr<ScenePart>(this);
    for(size_t i = 0 ; i < sp.mPrimitives.size() ; i++){
        p = sp.mPrimitives[i]->clone();
        p->part = thisShared;
        this->mPrimitives.push_back(p);
    }

    this->subpartLimit = subpartLimit;

}

// ***  M E T H O D S  *** //
// *********************** //
std::vector<Vertex*> ScenePart::getAllVertices() {
    std::vector<Vertex *> allPos;
    for (Primitive *p : mPrimitives) {
        for (size_t i = 0; i < p->getNumVertices(); i++) {
            allPos.push_back(p->getVertices()+i);
        }
    }
    return allPos;
}


void ScenePart::smoothVertexNormals(){
    Triangle *t;
    Vertex *v;
    std::map<Vertex *, std::shared_ptr<std::vector<Triangle *>>> vtmap;

    // Build map so for each vertex all triangles formed by it are known
    for(size_t i = 0 ; i < mPrimitives.size() ; i++){
        t = (Triangle *) mPrimitives[i];
        for(int j = 0 ; j <= 2 ; j++){
            if(!vtmap.count(t->verts + j)){
                std::shared_ptr<std::vector<Triangle *>> vec =
                    std::make_shared<std::vector<Triangle *>>();
                vtmap.insert({t->verts + j, vec});
            }
            vtmap[t->verts + j]->push_back(t);
        }
    }

    // Set the normal of each vertex as the mean of each triangle normal
    std::shared_ptr<std::vector<Triangle *>> vec;
    for(
        std::map<Vertex *, std::shared_ptr<std::vector<Triangle *>>>::iterator
            iter = vtmap.begin();
        iter != vtmap.end();
        iter++
    ){
        v = iter->first;
        vec = iter->second;
        v->normal[0] = 0.0;
        v->normal[1] = 0.0;
        v->normal[2] = 0.0;
        for(Triangle *t : *vec){
            v->normal += t->getFaceNormal();
        }
        v->normal = glm::normalize(v->normal);
    }
}

bool ScenePart::splitSubparts(){
    size_t n = subpartLimit.size();
    if(n <= 1) return false; // There is no need to do splits

    // Prepare incremental ID
    int start = -1;
    try{
        start = boost::lexical_cast<int>(mId);
    }
    catch(boost::bad_lexical_cast &blcex){
        std::stringstream ss;
        ss  << "Could not update subpart ID from \"" << mId << "\".\n "
            << "Thus, splitting subparts is aborted";
        logging::WARN(ss.str());
        return false;
    }

    // Do splits
    for(size_t i = 1 ; i < n ; ++i){
        std::shared_ptr<ScenePart> newPart = std::make_shared<ScenePart>();
        newPart->onRayIntersectionMode = onRayIntersectionMode;
        newPart->onRayIntersectionArgument = onRayIntersectionArgument;
        newPart->randomShift = randomShift;
        if(ladlut == nullptr) newPart->ladlut = nullptr;
        else newPart->ladlut = std::make_shared<LadLut>(*ladlut);
        newPart->mOrigin = mOrigin;
        newPart->mRotation = mRotation;
        newPart->mScale = mScale;
        for(size_t j = subpartLimit[i-1] ; j < subpartLimit[i] ; ++j){
            newPart->mPrimitives.push_back(mPrimitives[j]);
            mPrimitives[j]->part = newPart;
        }
        newPart->mId = std::to_string(start+i);
    }

    // Remove splitted primitives
    mPrimitives.erase(
        mPrimitives.begin() + subpartLimit[0],
        mPrimitives.end()
    );
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
