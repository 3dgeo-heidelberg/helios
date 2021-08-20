#ifdef PCL_BINDING
#include <demo/DemoSelector.h>
#include <util/HeliosException.h>

#include <sstream>

#include <demo/SimplePrimitivesDemo.h>
#include <demo/DynamicSceneDemo.h>

using HeliosDemos::DemoSelector;
using std::shared_ptr;
using std::make_shared;

// ***  SINGLETON: Instance  *** //
// ***************************** //
shared_ptr<DemoSelector> DemoSelector::ds = nullptr;

// ***  SINGLETON: Getter  *** //
// *************************** //
shared_ptr<DemoSelector> DemoSelector::getInstance(){
    if(ds == nullptr) ds = shared_ptr<DemoSelector>(new DemoSelector());
    return ds;
}

// ***  DEMO SELECTION METHOD  *** //
// ******************************* //
void HeliosDemos::DemoSelector::select(
    string const name,
    string const surveyPath,
    string const assetsPath
){
    using std::stringstream;

    // Handle demo
    if(name == "simple_primitives"){ // Handle simple primitives demo
        HeliosDemos::SimplePrimitivesDemo().run();
    }
    else if(name == "dynamic_scene"){  // Handle dynamic scene demo
        HeliosDemos::DynamicSceneDemo(surveyPath, assetsPath).run();
    }
    else{ // Handle unexpected demo
        stringstream ss;
        ss << "Demo selector failed to recognize '" << name << "' demo.";
        throw HeliosException(ss.str());
    }
}
#endif