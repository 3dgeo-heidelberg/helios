#ifdef PCL_BINDING

#include <visualhelios/VHCanvas.h>

#include <thread>
#include <chrono>


using visualhelios::VHCanvas;

using pcl::visualization::PCLVisualizer;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
VHCanvas::VHCanvas(string const title) : title(title) {
    viewer = PCLVisualizer::Ptr(new PCLVisualizer(title));
    timeBetweenUpdates = 100;
    forceRedraw = false;
}

// ***  CANVAS METHODS  *** //
// ************************ //
void VHCanvas::configure(){
    // Configure 3D viewer
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setSize(1024, 768);
}
void VHCanvas::start() {}
void VHCanvas::update() {}
void VHCanvas::postUpdate() {}
void VHCanvas::onStop(){
    std::cout   << "Visual helios canvas \"" << title
                << "\" has finised" << std::endl;
}
void VHCanvas::show(){
    configure();
    start();
    while(!viewer->wasStopped()){
        update();
        postUpdate();
        viewer->spinOnce(timeBetweenUpdates, forceRedraw);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(timeBetweenUpdates)
        );
    }
    onStop();
}

#endif