/**
 * threads.cpp
 *
 * Three versions of a shared counter incremented by two threads,
 * demonstrating
 * - version 1: unsynchronized access — data race, wrong result;
 * - version 2: std::mutex with std::lock_guard — correct, RAII locking;
 * - version 3: std::atomic — correct, lock-free.
 *
 * Compile: g++ -std=c++17 -pthread -o threads.out threads.cpp
 * Run:     ./threads.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#include <atomic>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>

static constexpr int ITERATIONS = 1'000'000;

/* --- Version 1: no synchronization --------------------------------------- */

int counter_v1 = 0;

/**
 * @brief Increments a shared counter with no protection.
 *
 * @param n Number of increments to perform.
 */
void increment_v1(int n)
{
  for (int i = 0; i < n; ++i) {
    counter_v1++; //! read-modify-write: NOT atomic
  }
}

/* --- Version 2: std::mutex + std::lock_guard ----------------------------- */

int counter_v2 = 0;
std::mutex mtx_v2;

/**
 * @brief Increments a shared counter under a lock_guard.
 *
 * @param n Number of increments to perform.
 */
void increment_v2(int n)
{
  for (int i = 0; i < n; ++i) {
    std::lock_guard<std::mutex> lock(mtx_v2);
    counter_v2++; //! protected by the mutex
  } //! lock_guard destroyed here — mutex unlocked (RAII)
}

/* --- Version 3: std::atomic ---------------------------------------------- */

std::atomic<int> counter_v3{0};

/**
 * @brief Increments an atomic counter — no explicit locking needed.
 *
 * @param n Number of increments to perform.
 */
void increment_v3(int n)
{
  for (int i = 0; i < n; ++i) {
    counter_v3++; //! atomic read-modify-write (single hardware instruction)
  }
}

/* --- Main ---------------------------------------------------------------- */

int main()
{
  int expected = 2 * ITERATIONS;

  //! --- Version 1: racy ---
  std::cout << "=== Version 1: no synchronization ===" << std::endl;
  {
    std::thread t1(increment_v1, ITERATIONS);
    std::thread t2(increment_v1, ITERATIONS);
    t1.join();
    t2.join();
  }
  std::cout << "  Expected: " << expected << std::endl;
  std::cout << "  Got:      " << counter_v1 << std::endl;
  std::cout << "  " << (counter_v1 == expected ? "OK" : "WRONG — data race!")
            << std::endl;

  std::cout << std::endl;

  //! --- Version 2: mutex ---
  std::cout << "=== Version 2: std::lock_guard ===" << std::endl;
  {
    std::thread t1(increment_v2, ITERATIONS);
    std::thread t2(increment_v2, ITERATIONS);
    t1.join();
    t2.join();
  }
  std::cout << "  Expected: " << expected << std::endl;
  std::cout << "  Got:      " << counter_v2 << std::endl;
  std::cout << "  " << (counter_v2 == expected ? "OK" : "WRONG")
            << std::endl;

  std::cout << std::endl;

  //! --- Version 3: atomic ---
  std::cout << "=== Version 3: std::atomic ===" << std::endl;
  {
    std::thread t1(increment_v3, ITERATIONS);
    std::thread t2(increment_v3, ITERATIONS);
    t1.join();
    t2.join();
  }
  std::cout << "  Expected: " << expected << std::endl;
  std::cout << "  Got:      " << counter_v3.load() << std::endl;
  std::cout << "  " << (counter_v3.load() == expected ? "OK" : "WRONG")
            << std::endl;

  exit(EXIT_SUCCESS);
}
