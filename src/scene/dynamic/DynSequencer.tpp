// ***  DYNAMIC SEQUENCING   *** //
// ***************************** //
template <typename T>
std::vector<std::shared_ptr<T>> DynSequencer<T>::nextStep(){
    // Check there is a sequence to operate over
    if(current == nullptr){
        throw HeliosException(
            "DynSequencer::nextStep failed because there was no sequence for "
            "next step"
        );
    }

    // Handle step
    std::vector<std::shared_ptr<T>> sequence = current->nextStep();

    // Prepare next step
    if(current->getIteration()>=current->getLoop() && current->getLoop()>0){
        // Advance to next sequence
        std::string nextId = current->getNext();
        if(nextId.empty()){ // No sequences left
            current = nullptr;
        }
        else{ // At least one more sequence left
            current = dynseqs[nextId];
            current->restart();
        }
    }

    // Return
    return sequence;
}

// ***   M E T H O D S   *** //
// ************************* //
template <typename T>
void DynSequencer<T>::release(){
    for(auto it = dynseqs.begin() ; it != dynseqs.end() ; ++it){
        it->second->clear();
    }
    dynseqs.clear();
    start = nullptr;
    current = nullptr;
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
template <typename T>
void DynSequencer<T>::add(std::shared_ptr<DynSequence<T>> dynseq){
    // Insert on map
    dynseqs.insert(std::pair<std::string, std::shared_ptr<DynSequence<T>>>(
        dynseq->getId(), dynseq
    ));

    // If no start, make it
    if(start == nullptr){
        start = dynseq;
        current = start;
    }
}

template <typename T>
std::shared_ptr<DynSequence<T>> DynSequencer<T>::get(std::string const &id){
    if(!has(id)) return nullptr;
    return dynseqs[id];
}
