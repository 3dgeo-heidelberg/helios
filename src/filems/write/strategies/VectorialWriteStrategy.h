#pragma once

#include <filems/write/strategies/WriteStrategy.h>

#include <vector>

namespace helios {
namespace filems {

using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing WriteStrategy to make any other
 *  write strategy operate over a vector
 * @tparam WriteArg The type of vectorial component
 * @tparam BiasType The types that define the bias
 */
template<typename WriteArg, typename... BiasType>
class VectorialWriteStrategy
  : public WriteStrategy<vector<WriteArg> const&, BiasType...>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The write strategy
   */
  WriteStrategy<WriteArg const&, BiasType...>& ws;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for vectorial write strategy
   */
  VectorialWriteStrategy(WriteStrategy<WriteArg const&, BiasType...>& ws)
    : ws(ws)
  {
  }
  virtual ~VectorialWriteStrategy() = default;

  // ***  WRITE STRATEGY INTERFACE  *** //
  // ********************************** //
  /**
   * @brief Write given vector
   * @param v The vector to be written
   */
  void write(vector<WriteArg> const& v, BiasType... bias) override
  {
    for (WriteArg const& vi : v)
      ws.write(vi, bias...);
  }
};

}
}
