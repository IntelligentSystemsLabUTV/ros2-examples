/**
 * Square plugin implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 29, 2022
 */

#include <cmath>

#include <polygons/square.hpp>

namespace Polygons
{

/**
 * @brief Initializes a Square object.
 *
 * @param side_length Side length to store.
 */
void POLYGONS_EXPORT Square::init(double side_length)
{
  side_length_ = side_length;
}

/**
 * @brief Returns the area of the current Square.
 *
 * @return Square area.
 */
double POLYGONS_EXPORT Square::area()
{
  return pow(side_length_, 2.0);
}

}  // namespace Polygons
