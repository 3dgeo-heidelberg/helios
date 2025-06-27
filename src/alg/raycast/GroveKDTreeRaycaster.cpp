#include <GroveKDTreeRaycaster.h>
#include <Primitive.h>

#include <vector>

// ***  GROVE DYNAMIC TREE METHODS  *** //
// ************************************ //
void
GroveKDTreeRaycaster::update(DynObject& dynObj)
{
  // Copy primitives
  std::shared_ptr<PointerVector<Primitive>> prims =
    sharedCopy(dynObj.mPrimitives);

  // Refresh cached primitives
  cache_prims = prims; // From now on, current primitives are cached

  // Make new tree with its independent set of primitives
  // To prevent they from being updated by other threads when raycasting
  root = std::shared_ptr<LightKDTreeNode>(
    kdtf->makeFromPrimitives(**prims, true, false));
  // TODO Pending : Be careful how many threads kdtf is using because this
  // method is called during simulation, when other threads might be running
}

std::shared_ptr<GroveKDTreeRaycaster>
GroveKDTreeRaycaster::makeTemporalClone() const
{
  std::shared_ptr<GroveKDTreeRaycaster> gkdtr =
    std::make_shared<GroveKDTreeRaycaster>(root, nullptr, cache_prims);
  return gkdtr;
}

std::shared_ptr<PointerVector<Primitive>>
GroveKDTreeRaycaster::sharedCopy(std::vector<Primitive*> const& src) const
{
  std::shared_ptr<PointerVector<Primitive>> _prims =
    std::make_shared<PointerVector<Primitive>>(src.size());
  std::vector<Primitive*>& prims = **_prims;
  for (Primitive* prim : src) {
    Primitive* clone = prim->clone();
    clone->part = prim->part;
    prims.push_back(clone);
  }
  return _prims;
}
