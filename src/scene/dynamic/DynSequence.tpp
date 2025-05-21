#ifndef _DYN_SEQUENCE_H_

#include <sstream>

using std::stringstream;

// ***  DYNAMIC SEQUENCING  *** //
// **************************** //
template <typename T>
vector<shared_ptr<T>> DynSequence<T>::nextStep(){
    if(loop == 0) return sequence;
    else if(iteration < loop){
        ++iteration;
        return sequence;
    }
    stringstream ss;
    ss << "DynSequence::nextStep failed because there were no remaining "
          "iterations\n\tCurrent iteration was " << (iteration+1) << " of "
          << loop;
    throw HeliosException(ss.str());
}

template <typename T>
void DynSequence<T>::restart(){
    iteration = 0;
}

#endif
