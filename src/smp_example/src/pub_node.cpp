/**
 * Publisher node for this example.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 26, 2021
 */

#include <chrono>

#include "../include/smp_example/smp_example.hpp"

/**
 * @brief Creates a PubNode.
 *
 * @param period Publishing timer time period.
 */
PubNode::PubNode(unsigned int period)
: Node("pub_node"),
  count_(0)
{
  count_pub_ = this->create_publisher<std_msgs::msg::UInt64>(
    "/ros2_examples/smp_topic",
    rclcpp::QoS(10)
  );
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period),
    std::bind(
      &PubNode::timer_clbk_,
      this
    )
  );
  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Publishes a new message.
 */
void PubNode::timer_clbk_(void)
{
  count_++;
  std_msgs::msg::UInt64 new_msg{};
  new_msg.set__data(count_);
  count_pub_->publish(new_msg);
}
