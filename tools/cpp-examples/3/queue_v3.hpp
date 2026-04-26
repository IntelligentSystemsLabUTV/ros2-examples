/**
 * queue_v3.hpp
 *
 * Solution to the Lecture 2 exercise.
 * The integer Queue has been evolved into
 * - a template class Queue<T> parameterized on the element type;
 * - internal storage replaced with std::vector<T> (no manual new/delete);
 * - instances managed via std::shared_ptr;
 * - an optional std::function callback invoked on each push().
 *
 * Starting point for the Lecture 3 exercise:
 *   1. Make the queue thread-safe by protecting all data access with
 *      std::mutex and std::lock_guard.
 *   2. Add a background std::thread that periodically invokes a registered
 *      reporting callback with the current queue size.
 *   3. Parameterize the reporting period with std::chrono::milliseconds.
 *   4. Use std::atomic<bool> as a stop flag for the background thread.
 *   5. Join the background thread in the destructor.
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#pragma once

#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>

/**
 * @brief A generic, fixed-capacity FIFO queue with an optional push callback.
 *
 * @tparam T Element type.
 */
template<typename T>
class Queue
{
public:
  //! Convenient alias — mirrors the pattern used throughout ROS 2
  using PushCallback = std::function<void(const T &)>;

  /**
   * @brief Constructor.
   *
   * @param capacity Maximum number of elements.
   */
  Queue(int capacity)
  : capacity_(capacity)
  {
    data_.reserve(capacity);
    std::cout << "Queue<T> created with capacity " << capacity_ << std::endl;
  }

  /**
   * @brief Destructor.
   */
  ~Queue()
  {
    std::cout << "Queue<T> destroyed" << std::endl;
  }

  /**
   * @brief Registers a callback to be invoked on every push.
   *
   * @param cb Callable with signature void(const T&).
   */
  void set_push_callback(PushCallback cb)
  {
    push_cb_ = cb;
  }

  /**
   * @brief Adds an element at the back of the queue.
   *
   * @param value Element to add.
   */
  void push(const T & value)
  {
    if (static_cast<int>(data_.size()) >= capacity_) {
      throw std::runtime_error("Queue is full");
    }
    data_.push_back(value);

    //! If a callback is registered, invoke it with the new element
    if (push_cb_) {
      push_cb_(value);
    }
  }

  /**
   * @brief Removes and returns the element at the front of the queue.
   *
   * @return Front element.
   */
  T pop()
  {
    if (data_.empty()) {
      throw std::runtime_error("Queue is empty");
    }
    T value = data_.front();
    data_.erase(data_.begin());
    return value;
  }

  /**
   * @brief Returns the current number of elements.
   *
   * @return Queue size.
   */
  inline int size() const { return static_cast<int>(data_.size()); }

  /**
   * @brief Checks whether the queue is empty.
   *
   * @return True if empty, false otherwise.
   */
  inline bool empty() const { return data_.empty(); }

private:
  int capacity_;
  std::vector<T> data_ = {};       //! no manual memory management — vector handles it
  PushCallback push_cb_ = nullptr; //! optional callback, empty by default
};
