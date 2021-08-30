#ifndef _DYN_SEQUENCER_H_

#include <scene/dynamic/DynSequence.h>

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

using std::string;
using std::unordered_map;
using std::vector;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @tparam T Type of elements composing sequences
 *
 * @brief Dynamic sequencer handles dynamic sequences in a sequential fashion
 *
 * @see DynSequence
 */
template <typename T>
class DynSequencer {
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a dynamic sequencer to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the dynamic sequencer
     */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        ar &dynseqs;
        ar &start;
        ar &current;
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The dynamic sequences handled by the dynamic sequencer
     */
    unordered_map<string, shared_ptr<DynSequence<T>>> dynseqs;
    /**
     * @brief The start dynamic sequence
     */
    shared_ptr<DynSequence<T>> start;
    /**
     * @brief The current dynamic sequence
     */
    shared_ptr<DynSequence<T>> current;
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for the dynamic sequencer
     */
    DynSequencer() : start(nullptr), current(nullptr) {}
    virtual ~DynSequencer() {}

    // ***  DYNAMIC SEQUENCING   *** //
    // ***************************** //
    /**
     * @brief Handle next sequential behavior
     * @return Sequence for the step
     */
    virtual vector<shared_ptr<T>> nextStep();
    /**
     * @brief Check if the dynamic sequencer supports a next step or not
     * @return True if dynamic sequencer supports a next step, false otherwise
     */
    virtual inline bool hasNextStep() {return current != nullptr;}

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Add the dynamic sequence to the sequencer.
     *
     * NOTICE the first added dynamic sequence will become the start dynamic
     *  sequence by default
     *
     * @param dynseq Dynamic sequence to be added
     */
    void add(shared_ptr<DynSequence<T>> dynseq);
    /**
     * @brief Remove the dynamic sequence with given identifier
     * @param id Identifier of dynamic sequence to be removed
     */
    inline void remove(string const & id) {dynseqs.erase(dynseqs.find(id));}
    /**
     * @brief Get the dynamic sequence with given identifier
     * @param id Identifier of dynamic sequence to be retrieved
     * @return Requested dynamic sequence if any, nullptr otherwise
     */
    shared_ptr<DynSequence<T>> get(string const &id);
    /**
     * @brief Check if the dynamic sequencer has a dynamic sequence with given
     *  identifier
     * @param id Identifier of the dynamic sequence to be checked
     * @return True if there is a dynamic sequence with given identifier, false
     *  otherwise
     */
    inline bool has(string const &id)
    {return dynseqs.find(id) != dynseqs.end();}
    /**
     * @brief Remove all dynamic sequences composing the dynamic sequencer
     */
    inline void clear() {dynseqs.clear();}
};

#include <scene/dynamic/DynSequencer.tpp>
#define _DYN_SEQUENCER_H_
#endif
