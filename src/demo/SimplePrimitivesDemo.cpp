#ifdef PCL_BINDING
#include <demo/SimplePrimitivesDemo.h>
#include <rigidmotion/RigidMotion.h>
#include <rigidmotion/RigidMotionR3Factory.h>
#include <rigidmotion/RigidMotionEngine.h>

#include <thread>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;
using std::vector;
using std::shared_ptr;
using std::make_shared;
using HeliosDemos::SimplePrimitivesDemo;
using rigidmotion::RigidMotion;
using rigidmotion::RigidMotionR3Factory;
using rigidmotion::RigidMotionEngine;

// ***  R U N  *** //
// *************** //
void SimplePrimitivesDemo::run(){
    std::cout << "RUNNING SIMPLE PRIMITIVES DEMO!" << std::endl; // TODO Remove

    // Open 3D viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Simple primitives demo viewer")
    );

    // Configure 3D viewer
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);  // Coordinate system unitary scaled
    viewer->initCameraParameters();

    // Visualize primitives
    vector<pcl::Vertices> mobileVertices, fixedVertices, staticVertices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mobileMesh = buildPolymesh(
        buildMobileStructure(),
        mobileVertices
    );
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixedMesh = buildPolymesh(
        buildFixedStructure(),
        fixedVertices
    );
    pcl::PointCloud<pcl::PointXYZ>::Ptr staticMesh = buildPolymesh(
        buildStaticStructure(),
        staticVertices
    );
    viewer->addPolygonMesh<pcl::PointXYZ>(
        mobileMesh,
        mobileVertices,
        "mobileMesh"
    );
    viewer->addPolygonMesh<pcl::PointXYZ>(
        fixedMesh,
        fixedVertices,
        "fixedMesh"
    );
    viewer->addPolygonMesh<pcl::PointXYZ>(
        staticMesh,
        staticVertices,
        "staticMesh"
    );

    // Initial positioning
    RigidMotionR3Factory rm3f;
    RigidMotionEngine rme;
    RigidMotion g = rm3f.makeRotationZ(M_PI/2);
    RigidMotion f = rm3f.makeTranslation(arma::colvec("-50;0;0"));
    f = rme.compose(f, g);
    arma::mat X(3, mobileMesh->size());
    for(size_t i=0; i < mobileMesh->size(); ++i){
        X(0, i) = mobileMesh->at(i).x;
        X(1, i) = mobileMesh->at(i).y;
        X(2, i) = mobileMesh->at(i).z;
    }
    arma::mat Y = rme.apply(f, X);
    for(size_t i=0; i < mobileMesh->size(); ++i){
        mobileMesh->at(i).x = Y(0, i);
        mobileMesh->at(i).y = Y(1, i);
        mobileMesh->at(i).z = Y(2, i);
    }
    viewer->updatePolygonMesh<pcl::PointXYZ>(
        mobileMesh,
        mobileVertices,
        "mobileMesh"
    );



    // Visualization loop
    double mobileTheta = 0.01;
    while(!viewer->wasStopped()){
        // Update objects
        f = rm3f.makeRotationZ(mobileTheta);
        X = arma::zeros(3, mobileMesh->size());
        for(size_t i=0; i < mobileMesh->size(); ++i){
            X(0, i) = mobileMesh->at(i).x;
            X(1, i) = mobileMesh->at(i).y;
            X(2, i) = mobileMesh->at(i).z;
        }
        Y = rme.apply(f, X);
        for(size_t i=0; i < mobileMesh->size(); ++i){
            mobileMesh->at(i).x = Y(0, i);
            mobileMesh->at(i).y = Y(1, i);
            mobileMesh->at(i).z = Y(2, i);
        }
        viewer->updatePolygonMesh<pcl::PointXYZ>(
            mobileMesh,
            mobileVertices,
            "mobileMesh"
        );
        //mobileTheta += 0.01;
        if(mobileTheta > 2*M_PI) mobileTheta = mobileTheta -
            ((int)(mobileTheta/2/M_PI))*2*M_PI;
        std::cout << "mobileTheta: " << mobileTheta << std::endl; // TODO Remove
        // TODO Rethirk : mobileMesh must be rotated wrt base position

        // Update screen
        viewer->spinOnce(100, true);
        std::this_thread::sleep_for(100ms);
    }
}

// ***  U T I L  *** //
// ***************** //
pcl::PointCloud<pcl::PointXYZ>::Ptr SimplePrimitivesDemo::buildPolymesh(
    vector<shared_ptr<Triangle>> const triangles,
    std::vector<pcl::Vertices> &vertices
){
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(
        new pcl::PointCloud<pcl::PointXYZ>()
    );
    int offset = 0;
    for(shared_ptr<Triangle> const & triangle : triangles){
        pcl::Vertices verts;
        int const n = (int) triangle->getNumVertices();
        for(int i = 0 ; i < n ; ++i){
            pcl::PointXYZ p(
                (float) triangle->verts[i].getX(),
                (float) triangle->verts[i].getY(),
                (float) triangle->verts[i].getZ()
            );
            pc->push_back(p);
            verts.vertices.push_back(offset+i);
        }
        vertices.push_back(verts);
        offset += triangle->getNumVertices();
    }
    return pc;
}

vector<shared_ptr<Triangle>> SimplePrimitivesDemo::buildMobileStructure(){
    vector<shared_ptr<Triangle>> triangles(0);
    // Bottom surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, 2.0, -1.0),
        Vertex(4.0, -2.0, -1.0),
        Vertex(-4.0, -2.0, -1.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, 2.0, -1.0),
        Vertex(4.0, 2.0, -1.0),
        Vertex(4.0, -2.0, -1.0)
    ));
    // Top surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, 2.0, 1.0),
        Vertex(4.0, -2.0, 1.0),
        Vertex(-4.0, -2.0, 1.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, 2.0, 1.0),
        Vertex(4.0, 2.0, 1.0),
        Vertex(4.0, -2.0, 1.0)
    ));
    // Left surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, -2.0, -1.0),
        Vertex(-4.0, -2.0, 1.0),
        Vertex(-4.0, 2.0, 1.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, -2.0, -1.0),
        Vertex(-4.0, 2.0, 1.0),
        Vertex(-4.0, 2.0, -1.0)
    ));
    // Right surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(4.0, -2.0, -1.0),
        Vertex(4.0, -2.0, 1.0),
        Vertex(4.0, 2.0, 1.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(4.0, -2.0, -1.0),
        Vertex(4.0, 2.0, 1.0),
        Vertex(4.0, 2.0, -1.0)
    ));
    // Front face
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, -2.0, -1.0),
        Vertex(-4.0, -2.0, 1.0),
        Vertex(4.0, -2.0, 1.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, -2.0, -1.0),
        Vertex(4.0, -2.0, 1.0),
        Vertex(4.0, -2.0, -1.0)
    ));
    // Front face
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, 2.0, -1.0),
        Vertex(-4.0, 2.0, 1.0),
        Vertex(4.0, 2.0, 1.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-4.0, 2.0, -1.0),
        Vertex(4.0, 2.0, 1.0),
        Vertex(4.0, 2.0, -1.0)
    ));
    return triangles;
}
vector<shared_ptr<Triangle>> SimplePrimitivesDemo::buildFixedStructure(){
    vector<shared_ptr<Triangle>> triangles(0);
    // Bottom surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, 2.0, -4.0),
        Vertex(2.0, -2.0, -4.0),
        Vertex(-2.0, -2.0, -4.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, 2.0, -4.0),
        Vertex(2.0, 2.0, -4.0),
        Vertex(2.0, -2.0, -4.0)
    ));
    // Top surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, 2.0, 4.0),
        Vertex(2.0, -2.0, 4.0),
        Vertex(-2.0, -2.0, 4.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, 2.0, 4.0),
        Vertex(2.0, 2.0, 4.0),
        Vertex(2.0, -2.0, 4.0)
    ));
    // Left surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, -2.0, -4.0),
        Vertex(-2.0, -2.0, 4.0),
        Vertex(-2.0, 2.0, 4.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, -2.0, -4.0),
        Vertex(-2.0, 2.0, 4.0),
        Vertex(-2.0, 2.0, -4.0)
    ));
    // Right surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(2.0, -2.0, -4.0),
        Vertex(2.0, -2.0, 4.0),
        Vertex(2.0, 2.0, 4.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(2.0, -2.0, -4.0),
        Vertex(2.0, 2.0, 4.0),
        Vertex(2.0, 2.0, -4.0)
    ));
    // Front face
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, -2.0, -4.0),
        Vertex(-2.0, -2.0, 4.0),
        Vertex(2.0, -2.0, 4.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, -2.0, -4.0),
        Vertex(2.0, -2.0, 4.0),
        Vertex(2.0, -2.0, -4.0)
    ));
    // Front face
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, 2.0, -4.0),
        Vertex(-2.0, 2.0, 4.0),
        Vertex(2.0, 2.0, 4.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-2.0, 2.0, -4.0),
        Vertex(2.0, 2.0, 4.0),
        Vertex(2.0, 2.0, -4.0)
    ));
    return triangles;
}
vector<shared_ptr<Triangle>> SimplePrimitivesDemo::buildStaticStructure(){
    vector<shared_ptr<Triangle>> triangles(0);
    // Bottom surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, 1.0, -7.0),
        Vertex(1.0, -1.0, -7.0),
        Vertex(-1.0, -1.0, -7.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, 1.0, -7.0),
        Vertex(1.0, 1.0, -7.0),
        Vertex(1.0, -1.0, -7.0)
    ));
    // Top surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, 1.0, 7.0),
        Vertex(1.0, -1.0, 7.0),
        Vertex(-1.0, -1.0, 7.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, 1.0, 7.0),
        Vertex(1.0, 1.0, 7.0),
        Vertex(1.0, -1.0, 7.0)
    ));
    // Left surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, -1.0, -7.0),
        Vertex(-1.0, -1.0, 7.0),
        Vertex(-1.0, 1.0, 7.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, -1.0, -7.0),
        Vertex(-1.0, 1.0, 7.0),
        Vertex(-1.0, 1.0, -7.0)
    ));
    // Right surface
    triangles.push_back(make_shared<Triangle>(
        Vertex(1.0, -1.0, -7.0),
        Vertex(1.0, -1.0, 7.0),
        Vertex(1.0, 1.0, 7.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(1.0, -1.0, -7.0),
        Vertex(1.0, 1.0, 7.0),
        Vertex(1.0, 1.0, -7.0)
    ));
    // Front face
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, -1.0, -7.0),
        Vertex(-1.0, -1.0, 7.0),
        Vertex(1.0, -1.0, 7.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, -1.0, -7.0),
        Vertex(1.0, -1.0, 7.0),
        Vertex(1.0, -1.0, -7.0)
    ));
    // Front face
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, 1.0, -7.0),
        Vertex(-1.0, 1.0, 7.0),
        Vertex(1.0, 1.0, 7.0)
    ));
    triangles.push_back(make_shared<Triangle>(
        Vertex(-1.0, 1.0, -7.0),
        Vertex(1.0, 1.0, 7.0),
        Vertex(1.0, 1.0, -7.0)
    ));
    return triangles;
}


#endif