/**
 * Multithreaded executors example.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 26, 2021
 */

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "../include/smp_example/smp_example.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage:\n\tsmp_example PERIOD[ms]" << std::endl;
    exit(EXIT_FAILURE);
  }
  unsigned int T = std::atoi(argv[1]);

  rclcpp::init(argc, argv);

  //! This type of executor is needed to handle multithreaded workloads
  //! coming from nodes
  rclcpp::executors::MultiThreadedExecutor smp_executor;

  //! So we explicitly need to add nodes to it
  auto smp_node = std::make_shared<SMPNode>();
  auto pub_node = std::make_shared<PubNode>(T);
  smp_executor.add_node(smp_node);
  smp_executor.add_node(pub_node);

  //! And then call its spin method directly
  smp_executor.spin();

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
