/**
 * queue_v2.hpp
 *
 * Solution to the Lecture 1 exercise.
 * The C-style queue is rewritten as
 * - a Queue class with RAII (constructor allocates, destructor deallocates);
 * - a PriorityQueue derived class that overrides push() with sorted insertion.
 *
 * Lecture 2 exercise:
 *   1. Templatize Queue to Queue<T>.
 *   2. Replace the internal raw array with std::vector<T>.
 *   3. Wrap instances in std::shared_ptr.
 *   4. Add an std::function<void(const T &)> callback invoked on each push().
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#pragma once

#include <stdexcept>

/**
 * @brief Fixed-capacity FIFO queue of integers, RAII-compliant.
 */
class Queue
{
public:
  /**
   * @brief Constructor: allocates the internal buffer.
   *
   * @param capacity Maximum number of elements.
   */
  Queue(int capacity);

  /**
   * @brief Destructor: deallocates the internal buffer.
   */
  virtual ~Queue(); //! virtual: safe deletion through base pointer

  /**
   * @brief Adds an element at the back of the queue.
   *
   * @param value Element to add.
   */
  virtual void push(int value); //! virtual: PriorityQueue will override this

  /**
   * @brief Removes and returns the element at the front of the queue.
   *
   * @return Front element.
   */
  int pop();

  /**
   * @brief Returns the current number of elements.
   *
   * @return Queue size.
   */
  inline int size() const { return count_; }

  /**
   * @brief Checks whether the queue is empty.
   *
   * @return True if empty, false otherwise.
   */
  inline bool empty() const { return count_ == 0; }

protected:
  //! protected, not private: derived classes need access to the internals
  int capacity_;
  int count_;
  int * data_;
};

/**
 * @brief A queue that keeps elements sorted in ascending order.
 *
 * push() inserts each element at the correct position so that
 * pop() always returns the smallest value (highest priority).
 */
class PriorityQueue : public Queue
{
public:
  /**
   * @brief Constructor: forwards capacity to the base class.
   *
   * @param capacity Maximum number of elements.
   */
  PriorityQueue(int capacity);

  /**
   * @brief Inserts an element in sorted (ascending) order.
   *
   * @param value Element to insert.
   */
  void push(int value) override;
  //! try to remove 'virtual' above and 'override' here, recompile and see what happens
};
