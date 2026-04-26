/**
 * shared_queue.cpp
 *
 * A producer/consumer pattern demonstrating
 * - std::thread with lambda callables;
 * - std::mutex and std::lock_guard for mutual exclusion;
 * - std::condition_variable with predicate for efficient waiting;
 * - std::chrono::milliseconds for timed delays;
 * - std::atomic<bool> as a stop flag.
 *
 * One producer thread pushes integers into a bounded buffer.
 * One consumer thread waits on a condition variable and pops them.
 * The producer signals completion via an atomic flag.
 *
 * Compile: g++ -std=c++17 -pthread -o shared_queue.out shared_queue.cpp
 * Run:     ./shared_queue.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

/* --- Shared state -------------------------------------------------------- */

static constexpr int CAPACITY = 4;

std::vector<int> buffer;              //! shared bounded buffer
std::mutex mtx;                       //! protects buffer
std::condition_variable cv_not_empty; //! signaled when buffer becomes non-empty
std::condition_variable cv_not_full;  //! signaled when buffer has room
std::atomic<bool> done{false};        //! producer sets this when finished

/* --- Producer ------------------------------------------------------------ */

/**
 * @brief Produces N items, pushing each into the shared buffer.
 *
 * @param n Number of items to produce.
 */
void producer(int n)
{
  for (int i = 1; i <= n; ++i) {
    //! Wait until the buffer has room
    {
      std::unique_lock<std::mutex> lock(mtx);
      cv_not_full.wait(lock, [&] {
        return static_cast<int>(buffer.size()) < CAPACITY;
      });
      buffer.push_back(i);
      std::cout << "  [producer] pushed " << i
                << "  (buffer size: " << buffer.size() << ")" << std::endl;
    } //! mutex released here

    cv_not_empty.notify_one(); //! wake the consumer

    //! Simulate some work
    std::this_thread::sleep_for(50ms);
  }

  //! Signal that no more items will be produced
  done.store(true);
  cv_not_empty.notify_one(); //! wake the consumer one last time
}

/* --- Consumer ------------------------------------------------------------ */

/**
 * @brief Consumes items from the shared buffer until the producer is done
 *        and the buffer is empty.
 */
void consumer()
{
  while (true) {
    int value;
    {
      std::unique_lock<std::mutex> lock(mtx);

      //! Wait until there is something to consume, OR the producer is done
      cv_not_empty.wait(lock, [&] {
        return !buffer.empty() || done.load();
      });

      //! If the buffer is empty and the producer is done, we are finished
      if (buffer.empty() && done.load()) {
        break;
      }

      value = buffer.front();
      buffer.erase(buffer.begin());
      std::cout << "  [consumer] popped " << value
                << "  (buffer size: " << buffer.size() << ")" << std::endl;
    } //! mutex released here

    cv_not_full.notify_one(); //! signal the producer that there is room

    //! Simulate some processing
    std::this_thread::sleep_for(120ms);
  }
}

/* --- Main ---------------------------------------------------------------- */

int main()
{
  static constexpr int N_ITEMS = 8;

  std::cout << "=== Producer/Consumer (capacity=" << CAPACITY
            << ", items=" << N_ITEMS << ") ===" << std::endl;

  std::thread prod(producer, N_ITEMS);
  std::thread cons(consumer);

  prod.join();
  cons.join();

  std::cout << std::endl;
  std::cout << "All items produced and consumed." << std::endl;

  exit(EXIT_SUCCESS);
}
