/**
 * Fibonacci computer action server main source file.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 10, 2022
 */

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "../include/actions_example/fib_server.hpp"

int main(int argc, char ** argv)
{
  std::cout << "Starting Fibonacci Computer action server..." << std::endl;
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<FibonacciComputer>();
#ifndef ADVANCED
  rclcpp::spin(server_node);
#else
  //! We need a MTExecutor for this
  rclcpp::executors::MultiThreadedExecutor smp_executor;
  smp_executor.add_node(server_node);
  smp_executor.spin();
#endif
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
