/**
 * rectangle.cpp
 *
 * Definitions of Rectangle member functions.
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 19, 2026
 */

#include "rectangle.hpp"

/**
 * @brief Constructor.
 *
 * @param width Width of the rectangle.
 * @param height Height of the rectangle.
 */
Rectangle::Rectangle(double width, double height)
: width_(width),
  height_(height)
{}

/**
 * @brief Computes the area of this rectangle.
 *
 * @return Area of the rectangle.
 */
double Rectangle::area() const
{
  return width_ * height_;
}

/**
 * @brief Returns the name of this shape.
 *
 * @return Name string "Rectangle".
 */
std::string Rectangle::name() const
{
  return "Rectangle";
}

/**
 * @brief Width getter.
 *
 * @return Width of the rectangle.
 */
double Rectangle::width() const
{
  return width_;
}

/**
 * @brief Height getter.
 *
 * @return Height of the rectangle.
 */
double Rectangle::height() const
{
  return height_;
}
