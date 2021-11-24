/**
 * Multithreaded executors example.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 24, 2021
 */

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "../include/smp_example/smp_node.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage:\n\tsmp_node T1 T2" << std::endl;
    exit(EXIT_FAILURE);
  }
  unsigned int T1 = std::atoi(argv[1]);
  unsigned int T2 = std::atoi(argv[2]);

  rclcpp::init(argc, argv);

  //! This type of executor is needed to handle multithreaded workloads
  //! coming from nodes
  rclcpp::executors::MultiThreadedExecutor smp_executor;

  //! So we explicitly need to add nodes to it, in this case just one
  auto smp_node = std::make_shared<SMPNode>(T1, T2);
  smp_executor.add_node(smp_node);

  //! And then call its spin method directly
  smp_executor.spin();

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
