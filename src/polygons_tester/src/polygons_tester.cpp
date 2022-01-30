/**
 * Polygons plugins test program.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 29, 2022
 */

#include <iostream>

//! Include pluginlib headers and base class header
#include <pluginlib/class_loader.hpp>
#include <polygon_base/polygon.hpp>

#define UNUSED(arg) (void)(arg)

int main(int argc, char ** argv)
{
  UNUSED(argc);
  UNUSED(argv);

  //! pluginlib throws exceptions when something fails
  try
  {
    //! First, create a ClassLoader providing package name and fully qualified
    //! base name
    pluginlib::ClassLoader<PolygonBase::Polygon> loader("polygon_base", "PolygonBase::Polygon");

    //! Then, initialize different objects as shared pointers to base class
    //! (other initializers are also possible, see the API)
    //! Specify the derived fully qualified name
    std::shared_ptr<PolygonBase::Polygon> triangle = loader.createSharedInstance("Polygons::Triangle");
    std::shared_ptr<PolygonBase::Polygon> square = loader.createSharedInstance("Polygons::Square");
    //! These wrap dlopen & co.

    //! Now you can do whatever you want with the objects!
    triangle->init(10.0);
    square->init(10.0);
    std::cout << "Triangle area: " << triangle->area() << std::endl;
    std::cout << "Square area: " << square->area() << std::endl;
  }
  catch(const pluginlib::PluginlibException & e)
  {
    std::cerr << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }
  //! Unfortunately exceptions may be thrown from the loader's creation going on,
  //! so this try-catch must wrap the code almost entirely
  //! If you don't do this and keep using objects outside then the loader will
  //! go out of scope outside the try block, and its destructor will attempt to
  //! unload the libraries, but it won't be able to since objects from them
  //! will still be around, so pluginlib will strongly complain

  exit(EXIT_SUCCESS);
}
