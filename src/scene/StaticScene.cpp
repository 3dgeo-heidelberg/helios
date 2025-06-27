#include <SerialIO.h>
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

// ***  READ/WRITE  *** //
// ******************** //
void
StaticScene::writeObject(std::string path)
{
  std::stringstream ss;
  ss << "Writing static scene object to " << path << " ...";
  logging::INFO(ss.str());
  SerialIO::getInstance()->write<StaticScene>(path, this);
}

StaticScene*
StaticScene::readObject(std::string path)
{
  std::stringstream ss;
  ss << "Reading static scene object from " << path << " ...";
  logging::INFO(ss.str());
  return SerialIO::getInstance()->read<StaticScene>(path);
}
