/**
 * Triangle plugin implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 29, 2022
 */

#include <cmath>

#include <polygons/triangle.hpp>

namespace Polygons
{

/**
 * @brief Initializes a Triangle object.
 *
 * @param side_length Side length to store.
 */
void POLYGONS_EXPORT Triangle::init(double side_length)
{
  side_length_ = side_length;
}

/**
 * @brief Returns the area of the current Triangle.
 *
 * @return Triangle area.
 */
double POLYGONS_EXPORT Triangle::area()
{
  return 0.5 * side_length_ * compute_height();
}

/**
 * @brief Computes the height of the current Triangle.
 *
 * @return Triangle height.
 */
double POLYGONS_IMPORT Triangle::compute_height()
{
  return sqrt((side_length_ * side_length_) - ((side_length_ / 2.0) * (side_length_ / 2.0)));
}

}  // namespace Polygons
