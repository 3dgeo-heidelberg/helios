#pragma once

#include "BaseTest.h"
#include <DetailedVoxel.h>
#include <Scene.h>
#include <SerialIO.h>
#include <vector>
#include <cstdio>

namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Test serialization
 */
class SerializationTest : public BaseTest{
public:
    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief Serialization test constructor
     */
    SerializationTest() : BaseTest("Serialization test"){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

    // ***  UTILS  *** //
    // *************** //
    /**
     * @brief Check two detailed voxels are equal
     * @param dv1 Written DetailedVoxel
     * @param dv2 Read DetailedVoxel
     * @return True if written and read detailed voxels are equal,
     * False otherwise
     */
    bool validate(DetailedVoxel &dv1, DetailedVoxel &dv2);
    /**
     * @brief Check two voxels are equal
     * @param v1 Written Voxel
     * @param v2 Read Voxel
     * @return True if written and read voxels are equal, False otherwise
     */
    bool validate(Voxel &v1, Voxel &v2);
    /**
     * @brief Check two axis aligned bounding boxes are equal
     * @param box1 Written AABB
     * @param box2 Read AABB
     * @return True if written and read axis aligned bounding boxes are equal,
     * False otherwise
     */
    bool validate(AABB &box1, AABB &box2);

    /**
     * @brief Check two triangles are equal
     * @param t1 Written Triangle
     * @param t2 Read Triangle
     * @return True if written and read triangles are equal, False otherwise
     */
    bool validate(Triangle &t1, Triangle &t2);
};

bool SerializationTest::run(){
    std::string path = "SerializationTest.tmp";

    // Detailed voxel serialization test
    DetailedVoxel dv1(
        1.0,
        2.0,
        1.0,
        1.0,
        std::vector<int>({0,1,2}),
        std::vector<double>({1.0, 1.5, 2.0, 2.5, 3.0})
    );
    dv1.part = std::make_shared<ScenePart>();
    dv1.part->onRayIntersectionMode = "TRANSMITTIVE";
    dv1.material = std::make_shared<Material>();
    dv1.material->ka[0] = 1.0;
    dv1.material->ka[0] = 2.0;
    dv1.material->ka[0] = 3.0;
    dv1.material->ka[0] = 4.0;
    SerialIO * sio = SerialIO::getInstance();
    sio->write<DetailedVoxel>(path, &dv1);
    SerialIO::getInstance()->write<DetailedVoxel>(path, &dv1);
    DetailedVoxel *dv2 = SerialIO::getInstance()->read<DetailedVoxel>(path);
    if(!validate(dv1,*dv2)) return false;

    // Scene serialization test
    size_t nRepeats = 32;
    Scene scene1;
    Vertex t1v1; t1v1.pos = glm::dvec3(0.0, 0.0, 0.0);
    Vertex t1v2; t1v2.pos = glm::dvec3(0.0, 0.0, 4.0);
    Vertex t1v3; t1v3.pos = glm::dvec3(2.0, 0.0, 0.0);
    Triangle t1(t1v1, t1v2, t1v3);
    Voxel v1(1.0, 1.0, 1.0, 1.0);
    AABB box1(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(5.0, 5.0, 5.0));
    scene1.primitives.push_back(dv1.clone());
    scene1.primitives.push_back(t1.clone());
    scene1.primitives.push_back(v1.clone());
    scene1.primitives.push_back(box1.clone());
    for(size_t i = 0 ; i < nRepeats ; i++)
        scene1.primitives.push_back(dv1.clone());
    scene1.primitives.push_back(dv1.clone());
    scene1.finalizeLoading();
    scene1.writeObject(path);
    Scene *scene2 = Scene::readObject(path);
    scene2->finalizeLoading();
    if(!validate(dv1, *(DetailedVoxel *) scene2->primitives[0])) return false;
    if(!validate(t1, *(Triangle *) scene2->primitives[1])) return false;
    if(!validate(v1, *(Voxel *) scene2->primitives[2])) return false;
    if(!validate(box1, *(AABB *) scene2->primitives[3])) return false;
    for(size_t i = 0 ; i < nRepeats ; i++){
        if(!validate(dv1, *(DetailedVoxel *) scene2->primitives[i+4]))
            return false;
    }


    // Remove temporary file
    std::remove(path.c_str());

    // Remove stuff allocated at serialization
    delete dv2;
    delete scene2;

    // Successfully reached end of test
    return true;
}

// ***  UTILS  *** //
// *************** //
bool SerializationTest::validate(
    DetailedVoxel &dv1, DetailedVoxel &dv2
){
    glm::dvec3 dv1c = dv1.getCentroid();
    glm::dvec3 dv2c = dv2.getCentroid();
    if(dv1c.x!=dv2c.x || dv1c.y!=dv2c.y || dv1c.z!=dv2c.z) return false;
    if(dv1.halfSize!=dv2.halfSize) return false;
    for(size_t i = 0 ; i < dv1.getNumberOfIntValues() ; i++){
        if(dv1.getIntValue(i)!=dv2.getIntValue(i)) return false;
    }
    for(size_t i = 0 ; i < dv1.getNumberOfDoubleValues() ; i++){
        if(dv1[i]!=dv2[i]) return false;
    }
    if(dv1.material == nullptr && dv2.material != nullptr) return false;
    if(dv1.material != nullptr && dv2.material == nullptr) return false;
    if(dv1.material != nullptr && dv2.material != nullptr) {
        for (size_t i = 0; i < 4; i++) {
            if (dv1.material->ka[i] != dv2.material->ka[i]) return false;
        }
    }
    return true;
}
bool SerializationTest::validate(
    Voxel  &v1, Voxel  &v2
){
    glm::dvec3 v1c = v1.getCentroid();
    glm::dvec3 v2c = v2.getCentroid();
    if(v1c.x!=v2c.x || v1c.y!=v2c.y || v1c.z!=v2c.z) return false;
    if(v1.halfSize!=v2.halfSize) return false;
    return true;
}
bool SerializationTest::validate(
    AABB  &box1, AABB  &box2
){
    double minX1 = box1.getMin().x;     double maxX1 = box1.getMax().x;
    double minY1 = box1.getMin().y;     double maxY1 = box1.getMax().y;
    double minZ1 = box1.getMin().z;     double maxZ1 = box1.getMax().z;
    double minX2 = box2.getMin().x;     double maxX2 = box2.getMax().x;
    double minY2 = box2.getMin().y;     double maxY2 = box2.getMax().y;
    double minZ2 = box2.getMin().z;     double maxZ2 = box2.getMax().z;

    if (minX1 != minX2) return false;
    if (minY1 != minY2) return false;
    if (minZ1 != minZ2) return false;

    if (maxX1 != maxX2) return false;
    if (maxY1 != maxY2) return false;
    if (maxZ1 != maxZ2) return false;

    return true;
}

bool SerializationTest::validate(Triangle &t1, Triangle &t2){
    double x1, x2, y1, y2, z1, z2;
    for(size_t i = 0 ; i < t1.getNumVertices() ; i++){
        x1 = t1.verts[i].getX();
        x2 = t2.verts[i].getX();
        if(x1 != x2) return false;
        y1 = t1.verts[i].getY();
        y2 = t2.verts[i].getY();
        if(y1 != y2) return false;
        z1 = t1.verts[i].getZ();
        z2 = t2.verts[i].getZ();
        if(z1 != z2) return false;
    }
    return true;
}




}
