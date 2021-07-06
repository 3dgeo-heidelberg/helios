//
// Created by miguelyermo on 5/7/21.
//

#pragma once

#include <vector>

// Forward declaration
class Primitive;

/**
 * @brief Class representing a .obj loaded file
 */
class WavefrontObj {
public:
  std::vector<Primitive*> primitives{};

  WavefrontObj() = default;
  WavefrontObj(WavefrontObj &obj);
  // TODO: Destructor
};
