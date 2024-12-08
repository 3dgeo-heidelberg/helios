#ifndef _DYN_SEQUENCE_H_

#include <string>
#include <vector>
#include <memory>

using std::string;
using std::vector;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @tparam T Type of elements composing the sequence
 *
 * @brief Dynamic sequence is a set of elements that must be applied during
 *  dynamic simulations to provide dynamic behavior to dynamic objects
 *
 * A dynamic sequence consists in a set of elements called the sequence
 *  \f$\mathcal{S}=\left\{S_1, \ldots, S_m\right\}\f$. The dynamic sequence
 *  is looped \f$l\f$ times. To illustrate this, let
 *  \f$f_t\left(\mathcal{S}, X\right)\f$ be the function which applies the
 *  sequence \f$\mathcal{S}\f$ to a dynamic object \f$X\f$ at \f$t\f$ time.
 *  Thus, applying a dynamic sequence can be summarized in following
 *  expression:
 *
 * \f[
 *  \forall t \in [1, l], f_t\left(\mathcal{S}, X\right)
 * \f]
 *
 * Once the sequence is applied, the sequence identified by the next attribute
 *  of finished sequence will be applied, if any.
 *
 * @see DynSequencer
 */
template <typename T>
class DynSequence {
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a dynamic sequence to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the dynamic sequence
     */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        ar &id;
        ar &next;
        ar &loop;
        ar &sequence;
        ar &iteration;
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Unique identifier for the dynamic sequence
     */
    string id;
    /**
     * @brief Unique identifier for dynamic sequence that must come after this
     *  one.
     *
     * NOTICE in case there is no sequence after this one, next should be an
     *  empty string ""
     */
    string next;
    /**
     * @brief Specify for how long the dynamic sequence must be repeated, using
     *  simulation steps as unit
     */
    size_t loop;
    /**
     * @brief The elements composing the dynamic sequence
     */
    vector<shared_ptr<T>> sequence;
    /**
     * @brief Current iteration. It is used to control the dynamic sequence
     *  application loop. By default, it must applied until iteration \f$=\f$
     *  loop
     */
    size_t iteration = 0;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for the dynamic sequence
     */
    DynSequence() : DynSequence("id", "", 1) {}
    /**
     * @brief Dynamic sequence basic constructor
     * @param id Identifier for the dynamic sequence
     * @param next Identifier of next dynamic sequence
     * @param loop For how many steps the dynamic sequence must be applied
     * @see DynSequence::id
     * @see DynSequence::next
     * @see DynSequence::loop
     */
    DynSequence(string id, string next, size_t loop) :
        id(id), next(next), loop(loop), iteration(0)
    {}
    virtual ~DynSequence() {}

    // ***  DYNAMIC SEQUENCING  *** //
    // **************************** //
    /**
     * @brief Obtain sequence corresponding to next step
     */
    vector<shared_ptr<T>> nextStep();
    /**
     * @brief Restart the dynamic sequence so when nextStep is called again
     *  it will start from the first iteration
     */
    void restart();

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Append an element to the sequence
     * @param element Element to be appended to the sequence
     * @see DynSequence::sequence
     */
    inline void append(shared_ptr<T> element) {sequence.push_back(element);}
    /**
     * @brief Append vector of elements to the end of the sequence.
     * @param elements Vector of elements to be appended to the end of the
     *  sequence
     * @see DynSequence::sequence
     */
    inline void append(vector<shared_ptr<T>> elements)
    {sequence.insert(sequence.end(), elements.begin(), elements.end());}
    /**
     * @brief Obtain element at given index in the sequence
     * @param index Index of element to be retrieved
     * @return Element at given index
     * @see DynSequence::sequence
     */
    inline shared_ptr<T> get(size_t index) const {return sequence[index];}
    /**
     * @brief Set element at given index in the sequence
     * @param index Index of element to be retrieved
     * @param element Element at given index
     * @see DynSequence::sequence
     */
    inline void set(size_t index, shared_ptr<T> element)
    {sequence[index = element];}
    /**
     * @brief Remove element at given index in the sequence
     * @param index Index of the element to be removed
     * @see DynSequence::sequence
     */
    inline void remove(size_t index)
    {sequence.erase(sequence.begin()+index);}
    /**
     * @brief Obtain the number of elements composing the sequence
     * @return Number of elements composing the sequence
     * @see DynSequence::sequence
     */
    inline size_t size() const {return sequence.size();}
    /**
     * @brief Obtain a iterator pointing to the first dynamic sequence element
     * @return Iterator pointing to the first dynamic sequence element
     */
    inline typename vector<T>::iterator begin() {return sequence.begin();}
    /**
     * @brief Obtain a iterator pointing to the last dynamic sequence element
     * @return Iterator pointing to the last dynamic sequence element
     */
    inline typename vector<T>::iterator end() {return sequence.end();}
    /**
     * @brief Remove all elements composing the sequence
     */
    inline void clear() {sequence.clear();}
    /**
     * @brief Get current iteration
     * @return Current iteration
     * @see DynSequence::iteration
     */
    inline size_t getIteration() const {return iteration;}
    /**
     * @brief Get loop value (max iterations)
     * @return Loop value (max iterations)
     * @see DynSequence::loop
     */
    inline size_t getLoop() const {return loop;}
    /**
     * @brief Set loop value (max iterations)
     * @param loop New loop value (max iterations)
     * @see DynSequence::loop
     */
    inline void setLoop(size_t const loop) {this->loop = loop;}
    /**
     * @brief Get current identifier for the dynamic sequence
     * @return Current identifier for the dynamic sequence
     * @see DynSequence::getId
     */
    inline string getId() const {return id;}
    /**
     * @brief Set identifier for the dynamic sequence
     * @param id New identifier for the dynamic sequence
     * @see DynSequence::setId
     */
    inline void setId(string const & id) {this->id = id;}
    /**
     * @brief Get next identifier for the dynamic sequence
     * @return Identifier of next dynamic sequence
     */
    inline string getNext() const {return next;}
    /**
     * @brief Set next identifier for the dynamic sequence
     * @param next Identifier for new next dynamic sequence
     */
    inline void setNext(string const & next) {this->next = next;}
};

#include <scene/dynamic/DynSequence.tpp>
#define _DYN_SEQUENCE_H_
#endif