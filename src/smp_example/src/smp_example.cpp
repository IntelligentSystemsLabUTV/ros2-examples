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

  rclcpp::executors::MultiThreadedExecutor smp_executor;

  auto smp_node = std::make_shared<SMPNode>(T1, T2);
  smp_executor.add_node(smp_node);

  smp_executor.spin();

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
