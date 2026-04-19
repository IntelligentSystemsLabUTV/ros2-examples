/**
 * queue.cpp — Version 1 (C-style)
 *
 * A fixed-capacity integer queue implemented as a struct with free functions.
 * This is deliberately written in a "C with iostream" style.
 *
 * YOUR TASK after C++ Lecture 1:
 *   1. Rewrite this as a proper C++ class with:
 *      - a constructor that takes the capacity and allocates the buffer;
 *      - a destructor that deallocates it (RAII);
 *      - push(), pop(), size(), empty() as member functions;
 *      - private data members.
 *   2. Add a derived class PriorityQueue that overrides push()
 *      to insert elements in sorted order (highest priority = lowest value
 *      at the front). Use virtual dispatch so that a Queue * pointing to a
 *      PriorityQueue calls the correct push().
 *
 * NOTE: This implementation is currently based on a circular buffer; to solve point 2, you are
 *       allowed to switch back to a linear buffer (an indexed array).
 *
 * Compile: g++ -std=c++17 -o queue queue.cpp
 * Run:     ./queue.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 19, 2026
 */

#include <cstdio>
#include <iostream>
#include <stdexcept>

//! This is a C-style struct: data members are public by default, no constructor/destructor.
/**
 * Simple implementation of a queue.
 */
struct Queue {
  int * data;
  int   capacity;
  int   head;  //! index of the front element
  int   tail;  //! index of the next free slot
  int   count; //! number of elements currently stored
};

/**
 * @brief Allocates and initializes a new queue.
 *
 * @param capacity Maximum number of elements.
 * @return Pointer to the new queue.
 */
Queue * queue_create(int capacity)
{
  Queue * q = new Queue;
  q->data     = new int[capacity];
  q->capacity = capacity;
  q->head     = 0;
  q->tail     = 0;
  q->count    = 0;
  return q;
}

/**
 * @brief Deallocates a queue and its internal buffer.
 *
 * @param q Pointer to the queue to destroy.
 */
void queue_destroy(Queue * q)
{
  delete[] q->data;
  delete q;
}

/**
 * @brief Enqueues a value at the back of the queue.
 *
 * @param q Pointer to the queue.
 * @param value Value to enqueue.
 */
void queue_push(Queue * q, int value)
{
  if (q->count >= q->capacity) {
    throw std::runtime_error("Queue is full");
  }
  q->data[q->tail] = value;
  q->tail = (q->tail + 1) % q->capacity;
  q->count++;
}

/**
 * @brief Dequeues and returns the front element.
 *
 * @param q Pointer to the queue.
 * @return Front element.
 */
int queue_pop(Queue * q)
{
  if (q->count == 0) {
    throw std::runtime_error("Queue is empty");
  }
  int value = q->data[q->head];
  q->head = (q->head + 1) % q->capacity;
  q->count--;
  return value;
}

/**
 * @brief Returns the number of elements currently in the queue.
 *
 * @param q Pointer to the queue.
 * @return Number of elements.
 */
int queue_size(const Queue * q)
{
  return q->count;
}

/**
 * @brief Checks whether the queue is empty.
 *
 * @param q Pointer to the queue.
 * @return True if empty, false otherwise.
 */
bool queue_empty(const Queue * q)
{
  return q->count == 0;
}

int main()
{
  Queue * q = queue_create(8);

  queue_push(q, 10);
  queue_push(q, 20);
  queue_push(q, 30);

  std::cout << "Size: " << queue_size(q) << "\n";  //! 3

  std::cout << "Popped: " << queue_pop(q) << "\n"; //! 10 (FIFO)
  std::cout << "Popped: " << queue_pop(q) << "\n"; //! 20

  std::cout << "Size after pops: " << queue_size(q) << "\n"; //! 1

  //! If we forget this call, we leak memory.
  //! With RAII, cleanup would be automatic.
  queue_destroy(q);

  exit(EXIT_SUCCESS);
}
