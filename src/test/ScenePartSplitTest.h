#pragma once

#include "BaseTest.h"
#include <ScenePart.h>
#include <Triangle.h>

#include <vector>

using std::vector;

namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Scene part split test
 */
class ScenePartSplitTest : public BaseTest{
public:
    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief Scene part split test constructor
     */
    ScenePartSplitTest() : BaseTest("Scene part split test"){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;
};


bool ScenePartSplitTest::run(){
    // Build primitives
    vector<Primitive *> prims;
    for(size_t i = 0 ; i < 32 ; ++i){
        Vertex v0, v1, v2;
        v0.pos = glm::dvec3(-1.0, -1.0, 0.0);
        v1.pos = glm::dvec3(0.0, 0.0, ((double)i)/32.0);
        v2.pos = glm::dvec3(1.0, 1.0, 0.0);
        Triangle *tr = new Triangle(v0, v1, v2);
        prims.push_back(tr);
    }

    // Build scene part
    std::shared_ptr<ScenePart> sp = std::make_shared<ScenePart>();
    for(size_t i = 0 ; i < 32 ; ++i) {
        sp->mPrimitives.push_back(prims[i]);
        prims[i]->part = sp;
        if (i > 0 && (i % 4) == 0) sp->subpartLimit.push_back(i);
    }
    sp->subpartLimit.push_back(32);
    sp->mId = "0";

    // Split scene part
    sp->splitSubparts();

    // Validate scene part splits
    for(int i = 0 ; i < 32 ; ++i){
        int partIdx = std::atoi(prims[i]->part->mId.c_str());
        if(partIdx != (i/4)){ // On test failed
            for(Primitive * prim : prims) delete prim;
            return false;
        }
    }

    // Delete primitives
    for(Primitive * prim : prims) delete prim;

    return true;
};
}