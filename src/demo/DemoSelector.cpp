#ifdef PCL_BINDING
#include <demo/DemoSelector.h>
#include <util/HeliosException.h>

#include <sstream>

#include <demo/DynamicSceneDemo.h>
#include <demo/RaycastingDemo.h>
#include <demo/SimplePrimitivesDemo.h>

using HeliosDemos::DemoSelector;
using std::make_shared;
using std::shared_ptr;

// ***  SINGLETON: Instance  *** //
// ***************************** //
shared_ptr<DemoSelector> DemoSelector::ds = nullptr;

// ***  SINGLETON: Getter  *** //
// *************************** //
shared_ptr<DemoSelector>
DemoSelector::getInstance()
{
  if (ds == nullptr)
    ds = shared_ptr<DemoSelector>(new DemoSelector());
  return ds;
}

// ***  DEMO SELECTION METHOD  *** //
// ******************************* //
void
HeliosDemos::DemoSelector::select(string const name,
                                  string const surveyPath,
                                  string const assetsPath)
{
  using std::stringstream;

  // Handle demo
  if (name == "simple_primitives") { // Handle simple primitives demo
    HeliosDemos::SimplePrimitivesDemo().run();
  } else if (name == "dynamic_scene") { // Handle dynamic scene demo
    HeliosDemos::DynamicSceneDemo(surveyPath, assetsPath).run();
  } else if (name == "raycasting") { // Handle ray casting demo
    HeliosDemos::RaycastingDemo(surveyPath, assetsPath).run();
  } else { // Handle unexpected demo
    stringstream ss;
    ss << "Demo selector failed to recognize '" << name << "' demo.";
    throw HeliosException(ss.str());
  }
}
#endif
