/**
 * circle.hpp
 *
 * Derived class demonstrating:
 * - public inheritance from Shape;
 * - override specifier.
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 19, 2026
 */

#pragma once

#include "shape.hpp"

/**
 * Representation of a circle.
 */
class Circle : public Shape
{
public:
  /**
   * @brief Constructor.
   *
   * @param radius Radius of the circle.
   */
  Circle(double radius);

  //! Note that 'virtual' is now replaced by 'override'.

  /**
   * @brief Computes the area of this circle.
   *
   * @return Area of the circle.
   */
  double area() const override;

  /**
   * @brief Returns the name of this shape.
   *
   * @return Name string "Circle".
   */
  std::string name() const override;

  /**
   * @brief Radius getter.
   *
   * @return Radius of the circle.
   */
  double radius() const;

private:
  double radius_;
};
