/**
 * shape.hpp
 *
 * Abstract base class demonstrating:
 * - pure virtual functions (= 0);
 * - virtual destructor;
 * - the class as an interface contract.
 *
 * Shape cannot be instantiated directly.
 * Derived classes must implement area() and name().
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 19, 2026
 */

#pragma once

#include <string>

/**
 * Abstract description of a shape.
 */
class Shape
{
public:
  //! No constructor => default constructor implemented by the compiler.
  //! Derived classes will then specify theirs.

  //! Virtual destructor: required for safe polymorphic deletion through a base-class pointer.
  virtual ~Shape() = default;

  /**
   * @brief Computes the area of this shape.
   *
   * @return Area of the shape.
   */
  virtual double area() const = 0; //! Pure virtual: every derived class must implement this.

  /**
   * @brief Returns the name of this shape.
   *
   * @return Name string.
   */
  virtual std::string name() const = 0; //! Pure virtual: every derived class must implement this.
};
