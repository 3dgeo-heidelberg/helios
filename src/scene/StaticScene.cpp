#include <scene/StaticScene.h>
#include <logging.hpp>
#include <SerialIO.h>

using std::stringstream;


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
StaticScene::StaticScene(StaticScene &ss) :
    StaticScene(static_cast<Scene &>(ss)
){
    for(shared_ptr<ScenePart> const &obj : ss.staticObjs){
        staticObjs.push_back(obj);
    }
}

// ***  READ/WRITE  *** //
// ******************** //
void StaticScene::writeObject(std::string path) {
    stringstream ss;
    ss << "Writing static scene object to " << path << " ...";
    logging::INFO(ss.str());
    SerialIO::getInstance()->write<StaticScene>(path, this);
}

StaticScene *StaticScene::readObject(std::string path) {
    stringstream ss;
    ss << "Reading static scene object from " << path << " ...";
    logging::INFO(ss.str());
    return SerialIO::getInstance()->read<StaticScene>(path);
}
