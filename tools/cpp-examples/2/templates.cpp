/**
 * templates.cpp
 *
 * Template basics demonstrating
 * - function templates with type deduction;
 * - class templates with multiple type parameters;
 * - explicit and deduced instantiation.
 *
 * Compile: g++ -std=c++17 -o templates.out templates.cpp
 * Run:     ./templates.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#include <cstdio>
#include <iostream>
#include <string>

/**
 * @brief Clamps a value between a minimum and a maximum.
 *
 * @param value Value to clamp.
 * @param lo    Lower bound.
 * @param hi    Upper bound.
 * @return Clamped value.
 */
template<typename T>
T clamp(T value, T lo, T hi)
{
  //! The compiler generates a separate version of this function for each T used.
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}

/**
 * @brief A generic pair of two (possibly different) types.
 */
template<typename A, typename B>
class Pair
{
public:
  /**
   * @brief Constructor: initializes both elements.
   *
   * @param first  First element.
   * @param second Second element.
   */
  Pair(A first, B second)
  : first_(first),
    second_(second)
  {}

  /**
   * @brief Swaps the two elements.
   *
   * Only available when A and B are the same type; the compiler will
   * refuse to instantiate this method otherwise.
   */
  void swap()
  {
    A tmp = first_;
    first_ = second_;
    second_ = tmp;
  }

  /**
   * @brief Prints both elements to stdout.
   */
  void print() const
  {
    std::cout << "(" << first_ << ", " << second_ << ")" << std::endl;
  }

private:
  A first_;
  B second_;
};

int main()
{
  //! --- Function template: clamp ---
  std::cout << "=== clamp ===" << std::endl;

  //! Explicit template argument
  std::cout << "clamp<int>(15, 0, 10):            "
            << clamp<int>(15, 0, 10) << std::endl;           //! 10

  //! Compiler deduces T = double from the arguments
  std::cout << "clamp(3.14, 0.0, 100.0):          "
            << clamp(3.14, 0.0, 100.0) << std::endl;         //! 3.14

  //! T = std::string — works because std::string supports < and >
  std::cout << "clamp(\"mango\", \"apple\", \"grape\"): "
            << clamp<std::string>("mango", "apple", "grape")
            << std::endl;                                     //! grape

  std::cout << std::endl;

  //! --- Class template: Pair ---
  std::cout << "=== Pair ===" << std::endl;

  Pair<int, int> p1(3, 7);
  std::cout << "p1 before swap: ";
  p1.print();   //! (3, 7)
  p1.swap();
  std::cout << "p1 after swap:  ";
  p1.print();   //! (7, 3)

  Pair<std::string, double> p2("pi", 3.14159);
  std::cout << "p2:             ";
  p2.print();   //! (pi, 3.14159)
  // p2.swap();
  //! p2.swap() would NOT compile: std::string and double are different types

  exit(EXIT_SUCCESS);
}
