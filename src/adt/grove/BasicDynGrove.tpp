#include <BasicDynGrove.h>
#include <BasicDynGroveSubject.h>

// ***  OBSERVER METHODS  *** //
// ************************** //
template <typename Tree, typename Subject>
void BasicDynGrove<Tree, Subject>::update(Subject &s){
    trees[s.getGroveSubjectId()]->update(s);
}

template <typename Tree, typename Subject>
void BasicDynGrove<Tree, Subject>::addSubject(
    BasicDynGroveSubject<Tree, Subject> *subject,
    std::shared_ptr<Tree> tree
){
    // Handle non-subject trees, if any (fill with nullptr)
    size_t const id = trees.size();
    size_t lastSubjectId = subjects.size();
    for(; lastSubjectId < id ; ++lastSubjectId) subjects.push_back(nullptr);

    // Add subject to the grove
    trees.push_back(tree);
    subjects.push_back(subject);
    if(subject != nullptr) subject->setGroveSubjectId(id);
}

template <typename Tree, typename Subject>
void BasicDynGrove<Tree, Subject>::removeSubject(
    BasicDynGroveSubject<Tree, Subject> *subject
){
    // Remove subject from grove
    size_t const id = subject->getGroveSubjectId();
    trees.erase(trees.begin() + id);
    subjects.erase(subjects.begin() + id);

    // Update identifier of remaining subjects
    BasicDynGroveSubject<Tree, Subject> *s;
    for(size_t i = id ; i < subjects.size() ; ++i){
        s = subjects[i];
        if(s!=nullptr) s->setGroveSubjectId(i);
    }
}
