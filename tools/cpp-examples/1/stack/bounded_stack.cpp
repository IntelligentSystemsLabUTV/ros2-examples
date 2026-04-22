/**
 * bounded_stack.cpp
 *
 * A fixed-capacity stack demonstrating:
 * - class definition with access specifiers
 * - constructor and destructor
 * - RAII: raw array allocated in constructor, deallocated in destructor
 * - const member functions
 * - scoped object lifetime
 *
 * Compile: g++ -std=c++17 -o bounded_stack bounded_stack.cpp
 * Run:     ./bounded_stack.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 19, 2026
 */

#include <cstdio>
#include <iostream>
#include <stdexcept>

/**
 * Simple, RAII-compliant implementation of a stack of integers.
 */
class BoundedStack
{
public:
  /**
   * @brief Constructor: acquires the resource (heap array).
   *
   * @param capacity Desired capacity.
   */
  BoundedStack(int capacity)
  : capacity_(capacity),
    size_(0),
    data_(new int[capacity])
  {
    //! Note the initializer list with which data members are initialized at the very start of the
    //! constructor body.
    std::cout << "Stack created with capacity " << capacity_ << std::endl;
  }

  /**
   * @brief Destructor: releases the resource.
   */
  ~BoundedStack()
  {
    delete[] data_;
    std::cout << "Stack destroyed, memory freed" << std::endl;
  }

  /**
   * @brief Adds a number to the top of the stack.
   *
   * @param value Number to add.
   */
  void push(int value)
  {
    if (size_ >= capacity_) {
      throw std::runtime_error("Stack is full");
    }
    data_[size_++] = value;
  }

  /**
   * @brief Removes a number from the top of the stack and returns it.
   *
   * @return Popped item.
   */
  int pop()
  {
    if (size_ == 0) {
      throw std::runtime_error("Stack is empty");
    }
    return data_[--size_];
  }

  /**
   * @brief Returns the number at the top of the stack without popping it.
   *
   * @return Number at the top of the stack.
   */
  int top() const
  {
    //! const: does not modify the object
    if (size_ == 0) {
      throw std::runtime_error("Stack is empty");
    }
    return data_[size_ - 1];
  }

  //! Now pay attention to the inline methods.

  /**
   * @brief Size getter.
   *
   * @return Stack size.
   */
  inline int size() const { return size_; }

  /**
   * @brief Checks if the stack is empty.
   *
   * @return True if the stack is empty, false otherwise.
   */
  inline bool empty() const { return size_ == 0; }

private:
  int capacity_;
  int size_;
  int * data_; //! Raw owning pointer — RAII handles cleanup
};

int main()
{
  //! Outer scope
  {
    //! Inner scope: the stack lives here
    BoundedStack s(4);

    s.push(10);
    s.push(20);
    s.push(30);

    std::cout << "Top: " << s.top() << std::endl;     //! 30
    std::cout << "Size: " << s.size() << std::endl;   //! 3

    std::cout << "Popped: " << s.pop() << std::endl;  //! 30
    std::cout << "Popped: " << s.pop() << std::endl;  //! 20

    std::cout << "Size after pops: " << s.size() << std::endl; // 1

    //! s goes out of scope here — destructor is called automatically
  }

  std::cout << "Back in outer scope — stack is gone" << std::endl;

  exit(EXIT_SUCCESS);
}
