/**
 * queue_v4.hpp
 *
 * Solution to the Lecture 3 exercise.
 * The template Queue<T> has been evolved into a thread-safe version:
 * - all data access protected with std::mutex and std::lock_guard;
 * - a background std::thread periodically invokes a reporting callback;
 * - the reporting period is parameterized with std::chrono::milliseconds;
 * - an std::atomic<bool> stop flag controls the background thread;
 * - the destructor joins the background thread (RAII).
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

/**
 * @brief A generic, fixed-capacity, thread-safe FIFO queue with optional
 *        push callback and periodic background reporting.
 *
 * @tparam T Element type.
 */
template<typename T>
class Queue
{
public:
  using PushCallback   = std::function<void(const T &)>;
  using ReportCallback = std::function<void(int)>;

  /**
   * @brief Constructor.
   *
   * @param capacity Maximum number of elements.
   */
  Queue(int capacity)
  : capacity_(capacity),
    stop_{false}
  {
    data_.reserve(capacity);
    std::cout << "Queue<T> created with capacity " << capacity_ << std::endl;
  }

  /**
   * @brief Destructor: stops the background thread and joins it.
   */
  ~Queue()
  {
    stop_reporter();
    std::cout << "Queue<T> destroyed" << std::endl;
  }

  /**
   * @brief Registers a callback to be invoked on every push.
   *
   * @param cb Callable with signature void(const T&).
   */
  void set_push_callback(PushCallback cb)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    push_cb_ = cb;
  }

  /**
   * @brief Starts a background thread that periodically reports the queue size.
   *
   * @param period  Reporting interval.
   * @param cb      Callable with signature void(int), receiving the current size.
   */
  void start_reporter(std::chrono::milliseconds period, ReportCallback cb)
  {
    //! Store the callback and clear the stop flag
    report_cb_ = cb;
    stop_.store(false);

    //! Launch the background thread
    reporter_ = std::thread(
      [this, period]() {
        while (!stop_.load()) {
          std::this_thread::sleep_for(period);

          if (stop_.load()) {
            break; //! check again after waking up
          }

          {
            std::lock_guard<std::mutex> lock(mtx_);
            if (report_cb_) {
              report_cb_(static_cast<int>(data_.size()));
            }
          }
        }
      });
  }

  /**
   * @brief Stops the background reporting thread, if running.
   */
  void stop_reporter()
  {
    if (reporter_.joinable()) {
      stop_.store(true);
      reporter_.join();
    }
  }

  /**
   * @brief Adds an element at the back of the queue (thread-safe).
   *
   * @param value Element to add.
   */
  void push(const T & value)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (static_cast<int>(data_.size()) >= capacity_) {
      throw std::runtime_error("Queue is full");
    }
    data_.push_back(value);

    //! If a push callback is registered, invoke it while still under the lock
    if (push_cb_) {
      push_cb_(value);
    }
  }

  /**
   * @brief Removes and returns the element at the front (thread-safe).
   *
   * @return Front element.
   */
  T pop()
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (data_.empty()) {
      throw std::runtime_error("Queue is empty");
    }
    T value = data_.front();
    data_.erase(data_.begin());
    return value;
  }

  /**
   * @brief Returns the current number of elements (thread-safe).
   *
   * @return Queue size.
   */
  int size() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return static_cast<int>(data_.size());
  }

  /**
   * @brief Checks whether the queue is empty (thread-safe).
   *
   * @return True if empty, false otherwise.
   */
  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_.empty();
  }

private:
  int capacity_;
  std::vector<T> data_;

  mutable std::mutex mtx_; //! mutable: lockable even in const methods

  PushCallback push_cb_;
  ReportCallback report_cb_;

  std::thread reporter_;
  std::atomic<bool> stop_;
};
