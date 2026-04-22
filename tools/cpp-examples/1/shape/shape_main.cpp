/**
 * shape_main.cpp
 *
 * Driver for the Shape hierarchy, demonstrating:
 * - polymorphic dispatch through base-class pointers;
 * - array of Shape * iterated uniformly;
 * - virtual destructor in action (delete through base pointer).
 *
 * Compile:
 *   g++ -std=c++17 -o shapes shape_main.cpp circle.cpp rectangle.cpp
 * Run:
 *   ./shapes.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 19, 2026
 */

#include <cstdio>
#include <iostream>

#include "circle.hpp"
#include "rectangle.hpp"

int main()
{
  //! An array of base-class pointers, each pointing to a different derived type.
  //! These objects are all allocated in the heap and initialized by calling their constructors,
  //! with given arguments.
  Shape * shapes[] = {
    new Circle(5.0),
    new Rectangle(4.0, 6.0),
    new Circle(1.5),
    new Rectangle(10.0, 2.0)
  };

  //! Polymorphic iteration: the correct override is called for each object.
  //! Also note the C++ iterator syntax used to loop through the array.
  for (const auto * s : shapes) {
    std::cout << s->name() << " — area: " << s->area() << "\n";
  }

  //! Polymorphic cleanup: virtual destructor ensures correct destruction through Shape *.
  for (auto * s : shapes) {
    delete s;
  }

  exit(EXIT_SUCCESS);
}
