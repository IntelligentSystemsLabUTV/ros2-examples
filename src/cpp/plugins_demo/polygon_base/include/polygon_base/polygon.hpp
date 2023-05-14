/**
 * Polygon abstract base class.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 28, 2022
 */

#ifndef POLYGON_BASE_POLYGON_HPP_
#define POLYGON_BASE_POLYGON_HPP_

//! All must be enclosed in a namespace for pluginlib to work
namespace PolygonBase
{

/**
 * Generic regular polygon.
 */
//! We want this to be an abstract class
class Polygon
{
public:
  //! Necessary since constructor can't have parameters!
  virtual void init(double side_length) = 0;
  virtual double area() = 0;
  //! If necessary, add a fini method

  //! Has to be virtual to comply with C++ specification
  virtual ~Polygon() {}

protected:
  //! This costructor signature is required by pluginlib
  Polygon() {}
};

}  // namespace PolygonBase

#endif
