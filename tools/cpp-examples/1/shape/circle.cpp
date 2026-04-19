/**
 * circle.cpp
 *
 * Definitions of Circle member functions.
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 19, 2026
 */

#include "circle.hpp"

//! C standard library math header
#include <cmath>

//! Note the ClassName:: qualifier on every function name.

/**
 * @brief Constructor.
 *
 * @param radius Radius of the circle.
 */
Circle::Circle(double radius)
: radius_(radius)
{}
//! The base class Shape is default-constructed implicitly via its default constructor.

/**
 * @brief Computes the area of this circle.
 *
 * @return Area of the circle.
 */
double Circle::area() const
{
  return M_PI * radius_ * radius_;
}

/**
 * @brief Returns the name of this shape.
 *
 * @return Name string "Circle".
 */
std::string Circle::name() const
{
  return "Circle";
}

/**
 * @brief Radius getter.
 *
 * @return Radius of the circle.
 */
double Circle::radius() const
{
  return radius_;
}
