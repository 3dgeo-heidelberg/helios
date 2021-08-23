#include <scene/StaticScene.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
StaticScene::StaticScene(StaticScene &ss) :
    StaticScene(static_cast<Scene &>(ss)
){
    for(shared_ptr<ScenePart> const &obj : ss.staticObjs){
        staticObjs.push_back(obj);
    }
}
