#include <sstream>

// ***  DYNAMIC SEQUENCING  *** //
// **************************** //
template<typename T>
std::vector<std::shared_ptr<T>>
DynSequence<T>::nextStep()
{
  if (loop == 0)
    return sequence;
  else if (iteration < loop) {
    ++iteration;
    return sequence;
  }
  std::stringstream ss;
  ss << "DynSequence::nextStep failed because there were no remaining "
        "iterations\n\tCurrent iteration was "
     << (iteration + 1) << " of " << loop;
  throw HeliosException(ss.str());
}

template<typename T>
void
DynSequence<T>::restart()
{
  iteration = 0;
}
