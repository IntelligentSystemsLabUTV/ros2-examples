/**
 * smart_pointers.cpp
 *
 * Shared pointer reference counting demonstrating
 * - std::make_shared and std::shared_ptr creation;
 * - reference count changes on copy, scope exit, and reset;
 * - automatic destruction when the count reaches zero.
 *
 * Compile: g++ -std=c++17 -o smart_pointers.out smart_pointers.cpp
 * Run:     ./smart_pointers.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

/**
 * @brief A trivial class that announces its own construction and destruction.
 */
class Probe
{
public:
  /**
   * @brief Constructor.
   *
   * @param tag A label to identify this instance.
   */
  Probe(const std::string & tag)
  : tag_(tag)
  {
    std::cout << "  Probe(\"" << tag_ << "\") constructed" << std::endl;
  }

  /**
   * @brief Destructor.
   */
  ~Probe()
  {
    std::cout << "  Probe(\"" << tag_ << "\") destroyed" << std::endl;
  }

  /**
   * @brief Returns the tag.
   *
   * @return Tag string.
   */
  inline const std::string & tag() const { return tag_; }

private:
  std::string tag_;
};

/**
 * @brief Prints the current reference count of a shared_ptr.
 *
 * @param label Description of the current step.
 * @param ptr   The shared pointer to inspect.
 */
void print_count(const std::string & label,
                 const std::shared_ptr<Probe> & ptr)
{
  std::cout << "  " << label << " — use_count = "
            << ptr.use_count() << std::endl;
}

int main()
{
  //! --- Creating a shared_ptr ---
  std::cout << "=== Creating a ===" << std::endl;
  auto a = std::make_shared<Probe>("alpha"); //! count = 1
  print_count("after creation", a);

  //! --- Copying increases the count ---
  std::cout << std::endl << "=== Copying into b ===" << std::endl;
  {
    auto b = a; //! count = 2
    print_count("a", a);
    print_count("b", b);

    //! --- A third copy ---
    std::cout << std::endl << "=== Copying into c ===" << std::endl;
    {
      auto c = a; //! count = 3
      print_count("a", a);
      print_count("c", c);
    }
    //! c goes out of scope here
    std::cout << std::endl << "=== c out of scope ===" << std::endl;
    print_count("a", a); //! count = 2
  }
  //! b goes out of scope here
  std::cout << std::endl << "=== b out of scope ===" << std::endl;
  print_count("a", a); //! count = 1

  //! --- Resetting the last owner (i.e. removing the ownership from a given shared_ptr) ---
  std::cout << std::endl << "=== Resetting a ===" << std::endl;
  a.reset(); //! count drops to 0 — Probe is destroyed here
  std::cout << "  a is now "
            << (a ? "valid" : "empty") << std::endl;

  exit(EXIT_SUCCESS);
}
