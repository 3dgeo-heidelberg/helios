#include <KDGrove.h>

// ***  KDGROVE METHODS  *** //
// ************************* //
std::shared_ptr<KDGrove> KDGrove::makeTemporalClone() const{
    std::shared_ptr<KDGrove> tkdg = std::make_shared<KDGrove>();
    std::vector<BasicDynGroveSubject<GroveKDTreeRaycaster, DynMovingObject> *>
        const &subjects = getSubjects();
    size_t const numTrees = getNumTrees();
    for(size_t i = 0 ; i < numTrees ; ++i){
        if(subjects[i] == nullptr){  // Tree for static object, reference copy
            tkdg->addTree(getTreeShared(i));
        }
        else{ // Tree for dynamic moving object, value copy
            tkdg->addTree(getTreeShared(i)->makeTemporalClone());
        }
    }
    return tkdg;
}
