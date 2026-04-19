/**
 * rectangle.hpp
 *
 * Derived class demonstrating:
 * - public inheritance from Shape;
 * - override specifier;
 * - constructor with multiple parameters.
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 19, 2026
 */

#pragma once

#include "shape.hpp"

/**
 * Representation of a rectangle.
 */
class Rectangle : public Shape
{
public:
  /**
   * @brief Constructor.
   *
   * @param width Width of the rectangle.
   * @param height Height of the rectangle.
   */
  Rectangle(double width, double height);

  /**
   * @brief Computes the area of this rectangle.
   *
   * @return Area of the rectangle.
   */
  double area() const override;

  /**
   * @brief Returns the name of this shape.
   *
   * @return Name string "Rectangle".
   */
  std::string name() const override;

  /**
   * @brief Width getter.
   *
   * @return Width of the rectangle.
   */
  double width() const;

  /**
   * @brief Height getter.
   *
   * @return Height of the rectangle.
   */
  double height() const;

private:
  double width_;
  double height_;
};
