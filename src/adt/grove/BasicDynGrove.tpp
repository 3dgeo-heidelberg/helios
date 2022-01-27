#include <BasicDynGrove.h>

// ***  OBSERVER METHODS  *** //
// ************************** //
template <typename Tree, typename Subject, typename SubjectId>
void BasicDynGrove<Tree, Subject, SubjectId>::update(Subject &s){
    observersMap[s.getId()].update(s);
}
