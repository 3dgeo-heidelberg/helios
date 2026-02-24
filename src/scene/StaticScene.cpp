#include <logging.hpp>
#include <scene/StaticScene.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
StaticScene::StaticScene(StaticScene& ss)
  : StaticScene(static_cast<Scene&>(ss))
{
  for (std::shared_ptr<ScenePart> const& obj : ss.staticObjs) {
    staticObjs.push_back(obj);
  }
}

// ***   M E T H O D S   *** //
// ************************* //
void
StaticScene::shutdown()
{
  Scene::shutdown();
  staticObjs.clear();
}
