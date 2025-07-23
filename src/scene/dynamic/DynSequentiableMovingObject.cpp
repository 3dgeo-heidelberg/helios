#include <scene/dynamic/DynSequentiableMovingObject.h>

#include <vector>

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
bool
DynSequentiableMovingObject::doSimStep()
{
  // Fill motion queues from sequencer
  fillMotionQueues();

  // Return DynMovingObject output
  return DynMovingObject::doSimStep();
}

void
DynSequentiableMovingObject::fillMotionQueues()
{
  // Fill motion queues from sequencer
  if (dmSequencer.hasNextStep()) { // If there is a next sequence
    // Iterate over its dynamic motions
    std::vector<std::shared_ptr<DynMotion>> sequence = dmSequencer.nextStep();
    for (std::shared_ptr<DynMotion>& dm : sequence) {
      pushPositionMotion(dm); // Push dynamic motion to position queue
      if (dm->checkModifiesNormal()) {
        // If dynamic motion modifies normal, push to normal queue too
        // Considering only the fixed transformation with zero shift
        pushNormalMotion(dm->makeNormalCounterpartPtr());
      }
    }
  }
}

// ***   U T I L S   *** //
// ********************* //
void
DynSequentiableMovingObject::applyAutoCRS(double const x,
                                          double const y,
                                          double const z)
{
  std::unordered_map<std::string,
                     std::shared_ptr<DynSequence<DynMotion>>> const& dynseqs =
    dmSequencer.getAllSequencesByRef();
  std::unordered_map<std::string,
                     std::shared_ptr<DynSequence<DynMotion>>>::const_iterator
    it;
  arma::colvec u({ x, y, z });
  for (it = dynseqs.begin(); it != dynseqs.end(); ++it) {
    std::shared_ptr<DynSequence<DynMotion>> ds = it->second;
    std::size_t const numMotions = ds->size();
    for (std::size_t i = 0; i < numMotions; ++i) {
      std::shared_ptr<DynMotion> dm = ds->get(i);
      if (dm->findType() == DynMotion::Type::TRANSLATION_R3 &&
          dm->isAutoCRS()) {
        dm->setC(dm->getC() + dm->getAutoCRS() * u);
      }
    }
  }
}

// ***   M E T H O D S   *** //
// ************************* //
void
DynSequentiableMovingObject::release()
{
  DynMovingObject::release();
  dmSequencer.release();
}
