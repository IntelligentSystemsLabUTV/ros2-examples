/**
 * queue_v4.cpp
 *
 * Driver for the thread-safe Queue<T>, demonstrating
 * - concurrent push/pop from multiple threads;
 * - background reporting thread with configurable period;
 * - push callback under lock;
 * - clean shutdown via destructor (RAII joins the reporter).
 *
 * Compile: g++ -std=c++17 -pthread -o queue_v4.out queue_v4.cpp
 * Run:     ./queue_v4.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <thread>

#include "queue_v4.hpp"

using namespace std::chrono_literals;

int main()
{
  std::cout << "=== Thread-safe Queue<int> ===" << std::endl;
  {
    auto q = std::make_shared<Queue<int>>(16);

    //! Register a push callback
    q->set_push_callback(
      [](const int & value) {
        std::cout << "  [push cb] pushed " << value << std::endl;
      });

    //! Start the background reporter: prints queue size every 200ms
    q->start_reporter(200ms,
      [](int sz) {
        std::cout << "  [reporter] size = " << sz << std::endl;
      });

    //! --- Producer thread: pushes 10 items with a short delay ---
    std::thread producer(
      [q]() {
        for (int i = 1; i <= 10; ++i) {
          q->push(i);
          std::this_thread::sleep_for(80ms);
        }
      });

    //! --- Consumer thread: pops items with a longer delay ---
    std::thread consumer(
      [q]() {
        //! Wait a bit so items accumulate first
        std::this_thread::sleep_for(300ms);
        for (int i = 0; i < 10; ++i) {
          //! Spin briefly if the queue is empty (simple approach for demo)
          while (q->empty()) {
            std::this_thread::sleep_for(50ms);
          }
          int val = q->pop();
          std::cout << "  [consumer] popped " << val << std::endl;
          std::this_thread::sleep_for(100ms);
        }
      });

    producer.join();
    consumer.join();

    //! Stop the reporter explicitly (also happens in destructor)
    q->stop_reporter();

    std::cout << std::endl;
    std::cout << "Final size: " << q->size() << std::endl;

    //! q goes out of scope — shared_ptr destroys the Queue, destructor joins
    //! the reporter thread if it hasn't been stopped already
  }

  exit(EXIT_SUCCESS);
}
