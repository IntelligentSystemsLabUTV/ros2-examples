/**
 * queue_v2.cpp
 *
 * Definitions and driver for Queue and PriorityQueue.
 *
 * Compile: g++ -std=c++17 -o queue_v2.out queue_v2.cpp
 * Run:     ./queue_v2.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#include <cstdio>
#include <iostream>

#include "queue_v2.hpp"

/* --- Queue ---------------------------------------------------------------- */

/**
 * @brief Constructor: allocates the internal buffer.
 *
 * @param capacity Maximum number of elements.
 */
Queue::Queue(int capacity)
: capacity_(capacity),
  count_(0),
  data_(new int[capacity])
{
  std::cout << "Queue created with capacity " << capacity_ << std::endl;
}

/**
 * @brief Destructor: deallocates the internal buffer.
 */
Queue::~Queue()
{
  delete[] data_;
  std::cout << "Queue destroyed, memory freed" << std::endl;
}

/**
 * @brief Adds an element at the back of the queue.
 *
 * @param value Element to add.
 */
void Queue::push(int value)
{
  if (count_ >= capacity_) {
    throw std::runtime_error("Queue is full");
  }
  data_[count_++] = value;
}

/**
 * @brief Removes and returns the element at the front of the queue.
 *
 * @return Front element.
 */
int Queue::pop()
{
  if (count_ == 0) {
    throw std::runtime_error("Queue is empty");
  }
  int value = data_[0];

  //! Shift all elements one position to the left
  for (int i = 1; i < count_; ++i) {
    data_[i - 1] = data_[i];
  }
  --count_;

  return value;
}

/* --- PriorityQueue -------------------------------------------------------- */

/**
 * @brief Constructor: forwards capacity to the base class.
 *
 * @param capacity Maximum number of elements.
 */
PriorityQueue::PriorityQueue(int capacity)
: Queue(capacity) //! explicit base-class constructor call
{}

/**
 * @brief Inserts an element in sorted (ascending) order.
 *
 * @param value Element to insert.
 */
void PriorityQueue::push(int value)
{
  if (count_ >= capacity_) {
    throw std::runtime_error("Queue is full");
  }

  //! Find the insertion point: first element greater than value
  int pos = 0;
  while (pos < count_ && data_[pos] <= value) {
    ++pos;
  }

  //! Shift elements to the right to make room
  for (int i = count_; i > pos; --i) {
    data_[i] = data_[i - 1];
  }

  data_[pos] = value;
  ++count_;
}

/* --- Main ----------------------------------------------------------------- */

int main()
{
  //! --- Regular Queue (FIFO) ---
  std::cout << "=== Queue (FIFO) ===" << std::endl;
  {
    Queue q(8);

    q.push(30);
    q.push(10);
    q.push(20);

    std::cout << "Size: " << q.size() << std::endl; //! 3

    std::cout << "Popped: " << q.pop() << std::endl; //! 30 (insertion order)
    std::cout << "Popped: " << q.pop() << std::endl; //! 10
    std::cout << "Popped: " << q.pop() << std::endl; //! 20

    //! q goes out of scope here — destructor frees the buffer
  }

  std::cout << "\n";

  //! --- PriorityQueue (sorted insertion) ---
  std::cout << "=== PriorityQueue ===" << std::endl;
  {
    PriorityQueue pq(8);

    pq.push(30);
    pq.push(10);
    pq.push(20);

    std::cout << "Size: " << pq.size() << std::endl; //! 3

    std::cout << "Popped: " << pq.pop() << std::endl; //! 10 (sorted order)
    std::cout << "Popped: " << pq.pop() << std::endl; //! 20
    std::cout << "Popped: " << pq.pop() << std::endl; //! 30
  }

  std::cout << "\n";

  //! --- Polymorphic dispatch through base pointer ---
  std::cout << "=== Polymorphic dispatch ===" << std::endl;
  {
    Queue * q = new PriorityQueue(8); //! base pointer, derived object

    q->push(50);
    q->push(5);
    q->push(25);

    //! The correct push() override was called — elements are sorted
    std::cout << "Popped: " << q->pop() << std::endl; //! 5
    std::cout << "Popped: " << q->pop() << std::endl; //! 25
    std::cout << "Popped: " << q->pop() << std::endl; //! 50

    //! Virtual destructor ensures PriorityQueue is destroyed correctly
    delete q;
  }

  exit(EXIT_SUCCESS);
}
