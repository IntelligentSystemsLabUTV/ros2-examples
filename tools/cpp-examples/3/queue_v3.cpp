/**
 * queue_v3.cpp
 *
 * Driver for Queue<T>, demonstrating
 * - template instantiation with different types;
 * - std::shared_ptr ownership and automatic cleanup;
 * - lambda callbacks registered on push.
 *
 * Compile: g++ -std=c++17 -o queue_v3.out queue_v3.cpp
 * Run:     ./queue_v3.out
 *
 * Roberto Masocco <roberto.masocco@uniroma2.it>
 *
 * April 26, 2026
 */

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "queue_v3.hpp"

int main()
{
  //! --- Queue<int> with a push callback ---
  std::cout << "=== Queue<int> with callback ===" << std::endl;
  {
    //! Create the queue via shared_ptr, as done throughout ROS 2
    auto q = std::make_shared<Queue<int>>(8);

    //! Register a callback that logs every push
    int push_count = 0;
    q->set_push_callback(
      [&push_count](const int & value) {
        ++push_count;
        std::cout << "  [callback] pushed " << value
                  << " (push #" << push_count << ")" << std::endl;
      });

    q->push(10);
    q->push(20);
    q->push(30);

    std::cout << "  Size: " << q->size() << std::endl;     //! 3
    std::cout << "  Popped: " << q->pop() << std::endl;    //! 10
    std::cout << "  Popped: " << q->pop() << std::endl;    //! 20
    std::cout << "  Size after pops: " << q->size() << std::endl; //! 1

    std::cout << "  Total pushes registered by callback: "
              << push_count << std::endl;

    //! q goes out of scope — shared_ptr destroys the Queue<int> automatically
  }

  std::cout << std::endl;

  //! --- Queue<std::string> without callback ---
  std::cout << "=== Queue<std::string> without callback ===" << std::endl;
  {
    auto q = std::make_shared<Queue<std::string>>(4);

    q->push("alpha");
    q->push("beta");
    q->push("gamma");

    std::cout << "  Size: " << q->size() << std::endl;     //! 3
    std::cout << "  Popped: " << q->pop() << std::endl;    //! alpha
    std::cout << "  Popped: " << q->pop() << std::endl;    //! beta
  }

  std::cout << std::endl;

  //! --- Shared ownership across multiple shared_ptrs ---
  std::cout << "=== Shared ownership ===" << std::endl;
  {
    auto owner1 = std::make_shared<Queue<int>>(4);
    owner1->push(99);

    std::cout << "  use_count after creation: "
              << owner1.use_count() << std::endl; //! 1

    {
      //! A second shared_ptr to the same queue
      auto owner2 = owner1;
      std::cout << "  use_count after copy: "
                << owner1.use_count() << std::endl; //! 2

      std::cout << "  owner2 pops: " << owner2->pop() << std::endl; //! 99
    }
    //! owner2 out of scope — count drops to 1, queue still alive
    std::cout << "  use_count after owner2 destroyed: "
              << owner1.use_count() << std::endl; //! 1
  }
  //! owner1 out of scope — count drops to 0, queue destroyed

  exit(EXIT_SUCCESS);
}
