#include <GroveTreeWrapper.h>
#include <StaticGrove.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
template <typename Tree>
GroveTreeWrapper<Tree>::GroveTreeWrapper(
    StaticGrove<Tree> &grove,
    size_t const index
) :
    grove(grove),
    index(index)
{}

// ***  FOR-EACH LOOP OPERATORS  *** //
// ********************************* //
template <typename Tree>
GroveTreeWrapper<Tree> GroveTreeWrapper<Tree>::operator++() {
    ++index;
    return *this;
}

template <typename Tree>
bool GroveTreeWrapper<Tree>::operator!=(
    GroveTreeWrapper<Tree> const &b
) const {
    return index != b.index;
}

template <typename Tree>
const std::shared_ptr<Tree> GroveTreeWrapper<Tree>::operator*() const {
    return grove[index];
}
