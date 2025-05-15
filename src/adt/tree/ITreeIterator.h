#pragma once

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Interface defining core mechanisms that must be provided by any
 *  tree iterator.
 *
 * @tparam NodeType  Type of tree node
 */
template<typename NodeType>
class ITreeIterator
{
private:
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a ITreeIterator to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the ITreeIterator
   */
  template<class Archive>
  void serialize(Archive& ar, unsigned int const version)
  {
  }

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for tree iterator
   */
  ITreeIterator() = default;
  virtual ~ITreeIterator() {}

  // ***  TREE ITERATOR INTERFACE  *** //
  // ********************************* //
  /**
   * @brief Start the iterator at given tree node
   * @param node Node to start iterator
   */
  virtual void start(NodeType node) = 0;
  /**
   * @brief Check if the iterator has more nodes to visit (true) or not
   *  (false)
   * @return True if there are nodes left to be visited, false otherwise
   */
  virtual bool hasNext() const = 0;
  /**
   * @brief Obtain the next node according to iterator criterion
   * @return Next node according to iterator criterion
   */
  virtual NodeType next() = 0;
};
