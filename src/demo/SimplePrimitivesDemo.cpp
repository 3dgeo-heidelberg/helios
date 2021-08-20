#ifdef PCL_BINDING

#include <demo/SimplePrimitivesDemo.h>
#include <rigidmotion/RigidMotion.h>
#include <rigidmotion/RigidMotionR3Factory.h>
#include <scene/dynamic/DynObject.h>
#include <scene/dynamic/DynMovingObject.h>
#include <visualhelios/VHSimpleCanvas.h>
#include <MathConstants.h>


using namespace std::chrono_literals;
using std::vector;
using std::shared_ptr;
using std::make_shared;
using HeliosDemos::SimplePrimitivesDemo;
using rigidmotion::RigidMotion;
using rigidmotion::RigidMotionR3Factory;
using visualhelios::VHSimpleCanvas;
using visualhelios::VHDynObjectXYZRGBAdapter;

// ***  R U N  *** //
// *************** //
void SimplePrimitivesDemo::run(){
    std::cout << "RUNNING SIMPLE PRIMITIVES DEMO ..." << std::endl;

    // Build objects
    shared_ptr<DynObject> mobileStructure = buildMobileStructure();
    shared_ptr<VHDynObjectXYZRGBAdapter> adaptedMobileStructure =
        make_shared<VHDynObjectXYZRGBAdapter>(*mobileStructure);
    adaptedMobileStructure->setRenderingNormals(true);
    shared_ptr<DynObject> fixedStructure = buildFixedStructure();
    shared_ptr<VHDynObjectXYZRGBAdapter> adaptedFixedStructure =
        make_shared<VHDynObjectXYZRGBAdapter>(*fixedStructure);
    adaptedFixedStructure->setRenderingNormals(true);
    shared_ptr<DynObject> helicalStructure = buildHelicalStructure();
    shared_ptr<VHDynObjectXYZRGBAdapter> adaptedHelicalStructure =
        make_shared<VHDynObjectXYZRGBAdapter>(*helicalStructure);
    adaptedHelicalStructure->setRenderingNormals(true);
    shared_ptr<DynObject> staticStructure = buildStaticStructure();
    shared_ptr<VHDynObjectXYZRGBAdapter> adaptedStaticStructure =
        make_shared<VHDynObjectXYZRGBAdapter>(*staticStructure);
    adaptedStaticStructure->setRenderingNormals(true);
    shared_ptr<DynObject> groundStructure = buildGroundStructure();
    shared_ptr<VHDynObjectXYZRGBAdapter> adaptedGroundStructure =
        make_shared<VHDynObjectXYZRGBAdapter>(*groundStructure);
    adaptedGroundStructure->setRenderingNormals(false);

    // Build canvas
    VHSimpleCanvas canvas("Simple primitives demo");
    canvas.appendDynObj(adaptedMobileStructure);
    canvas.appendDynObj(adaptedFixedStructure);
    canvas.appendDynObj(adaptedHelicalStructure);
    canvas.appendDynObj(adaptedStaticStructure);
    canvas.appendDynObj(adaptedGroundStructure);
    canvas.setTimeBetweenUpdates(20);
    canvas.setRenderingNormals(true);

    // Define initial transformations
    RigidMotionR3Factory rm3f;
    shared_ptr<DynMovingObject> dmoMobile =
        std::static_pointer_cast<DynMovingObject>(mobileStructure);
    RigidMotion rm = rm3f.makeRotationZ(PI_HALF);
    dmoMobile->pushPositionMotion(make_shared<RigidMotion>(rm));
    dmoMobile->pushNormalMotion(make_shared<RigidMotion>(rm));
    dmoMobile->pushPositionMotion(make_shared<RigidMotion>(
        rm3f.makeTranslation(arma::colvec("-40;0;1"))
    ));
    shared_ptr<DynMovingObject> dmoFixed =
        std::static_pointer_cast<DynMovingObject>(fixedStructure);
    dmoFixed->pushPositionMotion(make_shared<RigidMotion>(
        rm3f.makeTranslation(arma::colvec("10;10;4"))
    ));
    shared_ptr<DynMovingObject> dmoHelical =
        std::static_pointer_cast<DynMovingObject>(helicalStructure);
    rm = rm3f.makeRotationX(PI_HALF);
    dmoHelical->pushPositionMotion(make_shared<RigidMotion>(rm));
    dmoHelical->pushNormalMotion(make_shared<RigidMotion>(rm));
    dmoHelical->pushPositionMotion(make_shared<RigidMotion>(
        rm3f.makeTranslation(arma::colvec("0;5;1"))
    ));
    shared_ptr<DynMovingObject> dmoStatic =
        std::static_pointer_cast<DynMovingObject>(staticStructure);
    dmoStatic->pushPositionMotion(make_shared<RigidMotion>(
        rm3f.makeTranslation(arma::colvec("0;0;10"))
    ));

    // Define dynamic behavior function
    canvas.setDynamicUpdateFunction(
        [&rm3f](
            const vector<shared_ptr<VHDynObjectXYZRGBAdapter>>& objs
        ) -> void {
            DynMovingObject & dmoMobile = static_cast<DynMovingObject&>(
                objs[0]->getDynObj()
            );
            DynMovingObject & dmoFixed = static_cast<DynMovingObject&>(
                objs[1]->getDynObj()
            );
            DynMovingObject & dmoHelical = static_cast<DynMovingObject&>(
                objs[2]->getDynObj()
            );
            RigidMotion rm = rm3f.makeRotationZ(0.01);
            dmoMobile.pushPositionMotion(make_shared<RigidMotion>(rm));
            dmoMobile.pushNormalMotion(make_shared<RigidMotion>(rm));
            dmoFixed.pushPositionMotion(make_shared<RigidMotion>(
                rm3f.makeTranslation(arma::colvec("-10;-10;-4"))
            ));
            rm = rm3f.makeRotationZ(0.05);
            dmoFixed.pushPositionMotion(make_shared<RigidMotion>(rm));
            dmoFixed.pushNormalMotion(make_shared<RigidMotion>(rm));
            dmoFixed.pushPositionMotion(make_shared<RigidMotion>(
                rm3f.makeTranslation(arma::colvec("10;10;4"))
            ));
            if(dmoHelical.getPrimitives()[0]->getVertices()[0].getZ() >= 20.0)
                dmoHelical.pushPositionMotion(make_shared<RigidMotion>(
                    rm3f.makeTranslation(arma::colvec("0;0;-20"))
                ));
            dmoHelical.pushPositionMotion(make_shared<RigidMotion>(
                rm3f.makeHelicalZ(0.15, 0.034)
            ));
            dmoHelical.pushNormalMotion(make_shared<RigidMotion>(
                rm3f.makeRotationZ(0.15)
            ));
        }
    );

    // Render canvas
    canvas.show();

    // Destroy primitives
    vector<Primitive *> primitives = mobileStructure->getPrimitives();
    for(Primitive * primitive : primitives) delete primitive;
    primitives = fixedStructure->getPrimitives();
    for(Primitive * primitive : primitives) delete primitive;
    primitives = helicalStructure->getPrimitives();
    for(Primitive * primitive : primitives) delete primitive;
    primitives = staticStructure->getPrimitives();
    for(Primitive * primitive : primitives) delete primitive;
    primitives = groundStructure->getPrimitives();
    for(Primitive * primitive : primitives) delete primitive;

    std::cout << "FINISHED SIMPLE PRIMITIVES DEMO!" << std::endl;
}

// ***  U T I L  *** //
// ***************** //
shared_ptr<DynObject> SimplePrimitivesDemo::buildMobileStructure(){
    vector<Primitive *> triangles(0);
    // Bottom surface
    triangles.push_back(new Triangle(
        Vertex(-4.0, 2.0, -1.0),
        Vertex(4.0, -2.0, -1.0),
        Vertex(-4.0, -2.0, -1.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-4.0, 2.0, -1.0),
        Vertex(4.0, 2.0, -1.0),
        Vertex(4.0, -2.0, -1.0)
    ));
    // Top surface
    triangles.push_back(new Triangle(
        Vertex(-4.0, 2.0, 1.0),
        Vertex(-4.0, -2.0, 1.0),
        Vertex(4.0, -2.0, 1.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-4.0, 2.0, 1.0),
        Vertex(4.0, -2.0, 1.0),
        Vertex(4.0, 2.0, 1.0)
    ));
    // Left surface
    triangles.push_back(new Triangle(
        Vertex(-4.0, -2.0, -1.0),
        Vertex(-4.0, -2.0, 1.0),
        Vertex(-4.0, 2.0, 1.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-4.0, -2.0, -1.0),
        Vertex(-4.0, 2.0, 1.0),
        Vertex(-4.0, 2.0, -1.0)
    ));
    // Right surface
    triangles.push_back(new Triangle(
        Vertex(4.0, -2.0, -1.0),
        Vertex(4.0, 2.0, 1.0),
        Vertex(4.0, -2.0, 1.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(4.0, -2.0, -1.0),
        Vertex(4.0, 2.0, -1.0),
        Vertex(4.0, 2.0, 1.0)
    ));
    // Front face
    triangles.push_back(new Triangle(
        Vertex(-4.0, -2.0, -1.0),
        Vertex(4.0, -2.0, 1.0),
        Vertex(-4.0, -2.0, 1.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-4.0, -2.0, -1.0),
        Vertex(4.0, -2.0, -1.0),
        Vertex(4.0, -2.0, 1.0)
    ));
    // Back face
    triangles.push_back(new Triangle(
        Vertex(-4.0, 2.0, -1.0),
        Vertex(-4.0, 2.0, 1.0),
        Vertex(4.0, 2.0, 1.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-4.0, 2.0, -1.0),
        Vertex(4.0, 2.0, 1.0),
        Vertex(4.0, 2.0, -1.0)
    ));

    // Color and normals
    for(Primitive * primitive : triangles){
        ((Triangle *)primitive)->update(); // Compute normal
        ((Triangle *)primitive)->setAllVertexNormalsFromFace();
        Vertex * vertices = primitive->getVertices();
        for(size_t i = 0 ; i < primitive->getNumVertices() ; ++i){
            Color4f &color = vertices[i].color;
            color.x = 0.5;
            color.y = 0.0;
            color.z = 0.0;
        }
    }

    // Dynamic object
    shared_ptr<DynObject> dynObj = make_shared<DynMovingObject>(
        "mobileStructure",
        triangles
    );
    return dynObj;
}
shared_ptr<DynObject> SimplePrimitivesDemo::buildFixedStructure(){
    vector<Primitive *> triangles(0);
    // Bottom surface
    triangles.push_back(new Triangle(
        Vertex(-2.0, 2.0, -4.0),
        Vertex(2.0, -2.0, -4.0),
        Vertex(-2.0, -2.0, -4.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-2.0, 2.0, -4.0),
        Vertex(2.0, 2.0, -4.0),
        Vertex(2.0, -2.0, -4.0)
    ));
    // Top surface
    triangles.push_back(new Triangle(
        Vertex(-2.0, 2.0, 4.0),
        Vertex(-2.0, -2.0, 4.0),
        Vertex(2.0, -2.0, 4.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-2.0, 2.0, 4.0),
        Vertex(2.0, -2.0, 4.0),
        Vertex(2.0, 2.0, 4.0)
    ));
    // Left surface
    triangles.push_back(new Triangle(
        Vertex(-2.0, -2.0, -4.0),
        Vertex(-2.0, -2.0, 4.0),
        Vertex(-2.0, 2.0, 4.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-2.0, -2.0, -4.0),
        Vertex(-2.0, 2.0, 4.0),
        Vertex(-2.0, 2.0, -4.0)
    ));
    // Right surface
    triangles.push_back(new Triangle(
        Vertex(2.0, -2.0, -4.0),
        Vertex(2.0, 2.0, 4.0),
        Vertex(2.0, -2.0, 4.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(2.0, -2.0, -4.0),
        Vertex(2.0, 2.0, -4.0),
        Vertex(2.0, 2.0, 4.0)
    ));
    // Front face
    triangles.push_back(new Triangle(
        Vertex(-2.0, -2.0, -4.0),
        Vertex(2.0, -2.0, 4.0),
        Vertex(-2.0, -2.0, 4.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-2.0, -2.0, -4.0),
        Vertex(2.0, -2.0, -4.0),
        Vertex(2.0, -2.0, 4.0)
    ));
    // Front face
    triangles.push_back(new Triangle(
        Vertex(-2.0, 2.0, -4.0),
        Vertex(-2.0, 2.0, 4.0),
        Vertex(2.0, 2.0, 4.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-2.0, 2.0, -4.0),
        Vertex(2.0, 2.0, 4.0),
        Vertex(2.0, 2.0, -4.0)
    ));

    // Color and normals
    for(Primitive * primitive : triangles){
        Vertex * vertices = primitive->getVertices();
        ((Triangle *)primitive)->update(); // Compute normal
        ((Triangle *)primitive)->setAllVertexNormalsFromFace();
        for(size_t i = 0 ; i < primitive->getNumVertices() ; ++i){
            Color4f &color = vertices[i].color;
            color.x = 0.0;
            color.y = 0.5;
            color.z = 0.0;
        }
    }

    // Dynamic object
    shared_ptr<DynObject> dynObj = make_shared<DynMovingObject>(
        "fixedStructure",
        triangles
    );
    return dynObj;
}
shared_ptr<DynObject> SimplePrimitivesDemo::buildHelicalStructure(){
    vector<Primitive *> triangles(0);
    // Upper surface
    triangles.push_back(new Triangle(
        Vertex(-1, -1, 0),
        Vertex(1, -1, 0),
        Vertex(0, 0, 2)
    ));
    triangles.push_back(new Triangle(
        Vertex(0, 0, 2),
        Vertex(1, -1, 0),
        Vertex(0, 1, 0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-1, -1, 0),
        Vertex(0, 0, 2),
        Vertex(0, 1, 0)
    ));
    // Lower surface
    triangles.push_back(new Triangle(
        Vertex(-1, -1, 0),
        Vertex(0, 0, -2),
        Vertex(1, -1, 0)
    ));
    triangles.push_back(new Triangle(
        Vertex(0, 0, -2),
        Vertex(0, 1, 0),
        Vertex(1, -1, 0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-1, -1, 0),
        Vertex(0, 1, 0),
        Vertex(0, 0, -2)
    ));

    // Color and normals
    for(Primitive * primitive : triangles){
        ((Triangle *)primitive)->update(); // Compute normal
        ((Triangle *)primitive)->setAllVertexNormalsFromFace();
        Vertex * vertices = primitive->getVertices();
        for(size_t i = 0 ; i < primitive->getNumVertices() ; ++i){
            Color4f &color = vertices[i].color;
            color.x = 0.6;
            color.y = 0.6;
            color.z = 0.0;
        }
    }

    // Dynamic object
    shared_ptr<DynObject> dynObj = make_shared<DynMovingObject>(
        "helicalStructure",
        triangles
    );
    return dynObj;
}
shared_ptr<DynObject> SimplePrimitivesDemo::buildStaticStructure(){
    vector<Primitive *> triangles(0);
    // Bottom surface
    triangles.push_back(new Triangle(
        Vertex(-1.0, 1.0, -10.0),
        Vertex(1.0, -1.0, -10.0),
        Vertex(-1.0, -1.0, -10.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-1.0, 1.0, -10.0),
        Vertex(1.0, 1.0, -10.0),
        Vertex(1.0, -1.0, -10.0)
    ));
    // Top surface
    triangles.push_back(new Triangle(
        Vertex(-1.0, 1.0, 10.0),
        Vertex(-1.0, -1.0, 10.0),
        Vertex(1.0, -1.0, 10.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-1.0, 1.0, 10.0),
        Vertex(1.0, -1.0, 10.0),
        Vertex(1.0, 1.0, 10.0)
    ));
    // Left surface
    triangles.push_back(new Triangle(
        Vertex(-1.0, -1.0, -10.0),
        Vertex(-1.0, -1.0, 10.0),
        Vertex(-1.0, 1.0, 10.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-1.0, -1.0, -10.0),
        Vertex(-1.0, 1.0, 10.0),
        Vertex(-1.0, 1.0, -10.0)
    ));
    // Right surface
    triangles.push_back(new Triangle(
        Vertex(1.0, -1.0, -10.0),
        Vertex(1.0, 1.0, 10.0),
        Vertex(1.0, -1.0, 10.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(1.0, -1.0, -10.0),
        Vertex(1.0, 1.0, -10.0),
        Vertex(1.0, 1.0, 10.0)
    ));
    // Front face
    triangles.push_back(new Triangle(
        Vertex(-1.0, -1.0, -10.0),
        Vertex(1.0, -1.0, 10.0),
        Vertex(-1.0, -1.0, 10.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-1.0, -1.0, -10.0),
        Vertex(1.0, -1.0, -10.0),
        Vertex(1.0, -1.0, 10.0)
    ));
    // Back face
    triangles.push_back(new Triangle(
        Vertex(-1.0, 1.0, -10.0),
        Vertex(-1.0, 1.0, 10.0),
        Vertex(1.0, 1.0, 10.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(-1.0, 1.0, -10.0),
        Vertex(1.0, 1.0, 10.0),
        Vertex(1.0, 1.0, -10.0)
    ));

    // Color and normals
    for(Primitive * primitive : triangles){
        ((Triangle *)primitive)->update(); // Compute normal
        ((Triangle *)primitive)->setAllVertexNormalsFromFace();
        Vertex * vertices = primitive->getVertices();
        for(size_t i = 0 ; i < primitive->getNumVertices() ; ++i){
            Color4f &color = vertices[i].color;
            color.x = 0.2;
            color.y = 0.0;
            color.z = 0.4;
        }
    }

    // Dynamic object
    shared_ptr<DynObject> dynObj = make_shared<DynMovingObject>(
        "staticStructure",
        triangles
    );
    return dynObj;
}

shared_ptr<DynObject> SimplePrimitivesDemo::buildGroundStructure(){
    vector<Primitive *> triangles(0);
    // Ground surface
    triangles.push_back(new Triangle(
        Vertex(-50.0, -50.0, 0.0),
        Vertex(-50.0, 50.0, 0.0),
        Vertex(50.0, 50.0, 0.0)
    ));
    triangles.push_back(new Triangle(
        Vertex(50.0, 50.0, 0.0),
        Vertex(50.0, -50.0, 0.0),
        Vertex(-50.0, -50.0, 0.0)
    ));

    // Color
    for(Primitive * primitive : triangles){
        Vertex * vertices = primitive->getVertices();
        for(size_t i = 0 ; i < primitive->getNumVertices() ; ++i){
            Color4f &color = vertices[i].color;
            color.x = 0.5;
            color.y = 0.5;
            color.z = 0.5;
        }
    }

    // Dynamic object
    shared_ptr<DynObject> dynObj = make_shared<DynMovingObject>(
        "groundStructure",
        triangles
    );
    return dynObj;
}


#endif
