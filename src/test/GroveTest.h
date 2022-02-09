#pragma once

#include <BaseTest.h>
#include <KDGrove.h>
#include <GroveKDTreeRaycaster.h>
#include <Primitive.h>
#include <Triangle.h>
#include <FastSAHKDTreeFactory.h>

#include <vector>
#include <memory>

namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Test for grove of trees
 */
class GroveTest : public BaseTest{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The K-Dimensional Grove for testing purposes
     */
    KDGrove kdg;
    /**
     * @brief Trees defining
     */
    std::vector<std::shared_ptr<GroveKDTreeRaycaster>> trees;
    /**
     * @brief Primitives for each tree
     */
    std::vector<std::vector<Primitive *>> primitives;


public:
    // ***  CONSTRUCTOR  *** //
    // ********************* //
    GroveTest() : BaseTest("Grove test"){}
    virtual ~GroveTest(){
        // Release primitives
        for(std::vector<Primitive *> &triangles : primitives){
            for(Primitive * triangle : triangles){
                delete ((Triangle *)triangle);
            }
        }
    }

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

protected:
    // ***  SUB-TESTS  *** //
    // ******************* //
    /**
     * @brief Test the for, while and for-each loops of the grove
     * @return True if passed, false otherwise
     */
    bool testLoops();
    /**
     * @brief Test observer pattern mechanics are working as expected
     * @return True if passed, false otherwise
     */
    bool testObserving();

    // ***  UTILS  *** //
    // *************** //
    /**
     * @brief Build the primitives that will be used to define trees
     * @see HeliosTests::GroveTest::primitives
     */
    void buildPrimitives();
    /**
     * @brief Build the trees that will be contained in the grove
     * @see HeliosTests::GroveTest::trees
     */
    void buildTrees();
    /**
     * @brief Build the grove of trees to be tested
     * @see HeliosTests::GroveTest::kdg
     */
    void buildGrove();
};

// ***  R U N  *** //
// *************** //
bool GroveTest::run(){
    // Prepare tests
    buildPrimitives();
    buildTrees();
    buildGrove();

    // Do tests
    if(!testLoops()) return false;
    if(!testObserving()) return false;

    // All tests were successful
    return true;
}

// ***  SUB-TESTS  *** //
// ******************* //
bool GroveTest::testLoops(){
    // Prepare tests
    size_t i, j, iMax, m;

    // For loop test
    m = kdg.getNumTrees();
    for(j = 0 ; j < 3 ; ++j){ // Loop 3 times over grove
        for(i = 0, iMax = 0 ; i < m ; ++i){ // For loop over grove
            if(trees[i] != kdg[i]) return false;
            if(i > iMax) iMax = i;
        }
    }
    if(iMax!=m-1 || m!=trees.size()) return false;

    // While loop test
    for(j=0 ; j < 3 ; ++j){  // Loop 3 times over grove
        i = 0;
        kdg.toZeroTree(); // Start/restart while loop over grove
        while(kdg.hasNextTree()){ // While loop over grove
            if(trees[i] != kdg.nextTreeShared()) return false;
            ++i;
        }
    }
    if(i!=m) return false;

    // For-each loop test
    for(j=0 ; j < 3 ; ++j){ // Loop 3 times over grove
        i = 0;
        for(std::shared_ptr<GroveKDTreeRaycaster> gkdt : kdg){ // For-each grov
            if(trees[i] != gkdt) return false;
            ++i;
        }
    }
    if(i!=m) return false;

    // Loop tests passed
    return true;
}

bool GroveTest::testObserving(){
    // Prepare
    std::shared_ptr<GroveKDTreeRaycaster> tree = kdg.getTreeShared(0);
    DynMovingObject dmo1("dmo1");
    DynMovingObject dmo2("dmo2");

    // Add dmo1, dmo2 and validate
    kdg.addSubject(&dmo1, tree);
    kdg.addSubject(&dmo2, tree);
    if(((DynMovingObject *)kdg.getSubjects()[4])->getId() != "dmo1")
        return false;
    if(((DynMovingObject *)kdg.getSubjects()[5])->getId() != "dmo2")
        return false;

    // Remove dmo1 and validate
    kdg.removeSubject(&dmo1);
    if(((DynMovingObject *)kdg.getSubjects()[4])->getId() != "dmo2")
        return false;

    // Remove dmo2 and validate
    kdg.removeSubject(&dmo2);
    if(kdg.getSubjects().size() > 4) return false;

    // Observing test passed
    return true;
}

// ***  UTILS  *** //
// *************** //
void GroveTest::buildPrimitives(){
    // Primitives for first tree
    std::vector<Primitive *> triangles1;
    triangles1.push_back(new Triangle(
        Vertex(-1, -1, -1),
        Vertex(1, -1, -1),
        Vertex(0, 1, -1)
    ));
    triangles1.push_back(new Triangle(
        Vertex(-1, -1, -1),
        Vertex(1, -1, -1),
        Vertex(0, 0, 1)
    ));
    triangles1.push_back(new Triangle(
        Vertex(1, -1, -1),
        Vertex(0, 0, 1),
        Vertex(0, 1, -1)
    ));
    triangles1.push_back(new Triangle(
        Vertex(0, 1, -1),
        Vertex(0, 0, 1),
        Vertex(-1,-1, -1)
    ));
    primitives.push_back(triangles1);

    // Primitives for second tree
    std::vector<Primitive *> triangles2;
    triangles2.push_back(new Triangle(
        Vertex(0, 0, -1),
        Vertex(2, 0, -1),
        Vertex(1, 2, -1)
    ));
    triangles2.push_back(new Triangle(
        Vertex(0, 0, -1),
        Vertex(2, 0, -1),
        Vertex(1, 1, 1)
    ));
    triangles2.push_back(new Triangle(
        Vertex(2, 0, -1),
        Vertex(1, 1, 1),
        Vertex(1, 2, -1)
    ));
    triangles2.push_back(new Triangle(
        Vertex(1, 2, -1),
        Vertex(1, 1, 1),
        Vertex(0, 0, -1)
    ));
    primitives.push_back(triangles2);

    // Primitives for third tree
    std::vector<Primitive *> triangles3;
    triangles3.push_back(new Triangle(
        Vertex(4, 4, -1),
        Vertex(6, 4, -1),
        Vertex(5, 6, -1)
    ));
    triangles3.push_back(new Triangle(
        Vertex(4, 4, -1),
        Vertex(6, 4, -1),
        Vertex(5, 5, 1)
    ));
    triangles3.push_back(new Triangle(
        Vertex(6, 4, -1),
        Vertex(5, 5, 1),
        Vertex(5, 6, -1)
    ));
    triangles3.push_back(new Triangle(
        Vertex(5, 6, -1),
        Vertex(5, 5, 1),
        Vertex(4, 4, -1)
    ));
    primitives.push_back(triangles3);

    // Primitives for fourth tree
    std::vector<Primitive *> triangles4;
    triangles4.push_back(new Triangle(
        Vertex(9, 9, 0),
        Vertex(11, 9, 0),
        Vertex(10, 11, 0)
    ));
    triangles4.push_back(new Triangle(
        Vertex(9, 9, 0),
        Vertex(11, 9, 0),
        Vertex(10, 10, -3)
    ));
    triangles4.push_back(new Triangle(
        Vertex(11, 9, 0),
        Vertex(10, 10, -3),
        Vertex(10, 11, 0)
    ));
    triangles4.push_back(new Triangle(
        Vertex(10, 11, 0),
        Vertex(10, 10, -3),
        Vertex(9, 9, 0)
    ));
    primitives.push_back(triangles4);
}
void GroveTest::buildTrees(){
    FastSAHKDTreeFactory kdtf(32, 1, 1, 1);
    std::shared_ptr<LightKDTreeNode> tree1(
        kdtf.makeFromPrimitives(primitives[0], true, false)
    );
    trees.push_back(std::make_shared<GroveKDTreeRaycaster>(tree1));
    std::shared_ptr<LightKDTreeNode> tree2(
        kdtf.makeFromPrimitives(primitives[1], true, false)
    );
    trees.push_back(std::make_shared<GroveKDTreeRaycaster>(tree2));
    std::shared_ptr<LightKDTreeNode> tree3(
        kdtf.makeFromPrimitives(primitives[2], true, false)
    );
    trees.push_back(std::make_shared<GroveKDTreeRaycaster>(tree3));
    std::shared_ptr<LightKDTreeNode> tree4(
        kdtf.makeFromPrimitives(primitives[3], true, false)
    );
    trees.push_back(std::make_shared<GroveKDTreeRaycaster>(tree4));
}
void GroveTest::buildGrove(){
    for(std::shared_ptr<GroveKDTreeRaycaster> gkdtrc : trees){
        kdg.addTree(gkdtrc);
    }
}

}