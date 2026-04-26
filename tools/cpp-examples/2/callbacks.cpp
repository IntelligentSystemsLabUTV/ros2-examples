/**
 * callbacks.cpp
 *
 * Callbacks demonstrating
 * - lambdas with capture by value, by reference, and no capture;
 * - a plain function pointer stored alongside lambdas;
 * - std::function as a callable container;
 * - registering and invoking a vector of callbacks.
 *
 * Compile: g++ -std=c++17 -o callbacks.out callbacks.cpp
 * Run:     ./callbacks.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#include <cstdio>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

/**
 * @brief A plain function — not a lambda, not a functor.
 *
 * @param value The value received from the caller.
 */
void plain_printer(int value)
{
  std::cout << "  [plain function]  value = " << value << std::endl;
}

int main()
{
  //! A vector of callbacks: each element can hold any callable with
  //! signature void(int).
  std::vector<std::function<void(int)>> callbacks;

  //! --- 1. Lambda capturing a local variable by value ---
  std::string tag = "snapshot";
  callbacks.push_back(
    [tag](int value) {
      //! tag is a COPY taken at the moment of capture
      std::cout << "  [by value, tag=\"" << tag << "\"]  value = "
                << value << std::endl;
    });

  //! --- 2. Lambda capturing a local variable by reference ---
  int call_count = 0;
  callbacks.push_back(
    [&call_count](int value) {
      //! call_count is the ORIGINAL variable — modifications are visible outside
      ++call_count;
      std::cout << "  [by reference, call #" << call_count << "]  value = "
                << value << std::endl;
    });

  //! --- 3. Plain function pointer ---
  callbacks.push_back(&plain_printer);

  //! --- Modify the captured variables AFTER registration ---
  tag = "modified";
  //! The by-value lambda still sees "snapshot" — it has its own copy.
  //! The by-reference lambda will see the current call_count.

  //! --- Invoke all callbacks ---
  std::cout << "=== First round (value = 42) ===" << std::endl;
  for (auto & cb : callbacks) {
    cb(42);
  }

  std::cout << std::endl;

  std::cout << "=== Second round (value = 99) ===" << std::endl;
  for (auto & cb : callbacks) {
    cb(99);
  }

  std::cout << std::endl;

  //! call_count was captured by reference: it has been incremented twice
  std::cout << "call_count after two rounds: " << call_count << std::endl;

  exit(EXIT_SUCCESS);
}
