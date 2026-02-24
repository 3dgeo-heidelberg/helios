#pragma once

/**
 * @brief Class representing a color with 4 float components: RGBA
 */
class Color4f
{
private:
  // *********************** //

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Color red component (R)
   */
  float x = 1;
  /**
   * @brief Color green component (G)
   */
  float y = 1;
  /**
   * @brief Color blue component (B)
   */
  float z = 1;
  /**
   * @brief Color alpha component (A)
   */
  float w = 1;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Color in 4 float components default constructor
   */
  Color4f() = default;

  /**
   * @brief Color in 4 float components constructor
   * @see Color4f::x
   * @see Color4f::y
   * @see Color4f::z
   * @see Color4f::w
   */
  Color4f(float x, float y, float z, float w)
  {
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
  }
};
