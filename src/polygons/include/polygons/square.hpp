/**
 * Square plugin structure.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 29, 2022
 */

#ifndef POLYGONS__SQUARE_HPP_
#define POLYGONS__SQUARE_HPP_

//! Necessary to register plugin classes with pluginlib
#include <pluginlib/class_list_macros.hpp>

//! Include base class header and visibility control macros
#include <polygon_base/polygon.hpp>
#include <polygons/visibility_control.h>

//! Namespaces must reflect what is written in plugins.xml!
namespace Polygons
{

/**
 * Regular polygon with four sides.
 */
class POLYGONS_PUBLIC Square : public PolygonBase::Polygon
{
//! Stuff in here must be like in the base class: virtual methods must be
//! defined, then one can add whatever one wants, but only what is defined
//! in the base class will be visible!
//! Member functions from the base class must be overridden, since we'll
//! access them through a pointer to the base class
public:
  void init(double side_length) override;
  double area() override;

//! There must not be any private members
protected:
  double side_length_;
};

}  // namespace Polygons

#endif  // POLYGONS__SQUARE_HPP_

//! Register this class for pluginlib, providing derived and base fully
//! qualified names
PLUGINLIB_EXPORT_CLASS(Polygons::Square, PolygonBase::Polygon)
