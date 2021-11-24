#include <chrono>
#include <thread>

#include "../include/smp_example/smp_node.hpp"

SMPNode::SMPNode(unsigned int T1, unsigned int T2)
: Node("smp_node")
{
  clbk_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clbk_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer1_ = this->create_wall_timer(
    std::chrono::milliseconds(T1),
    std::bind(
      &SMPNode::timer_clbk_1,
      this
    ),
    clbk_group_1_
  );

  timer2_ = this->create_wall_timer(
    std::chrono::milliseconds(T2),
    std::bind(
      &SMPNode::timer_clbk_2,
      this
    ),
    clbk_group_2_
  );

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

void SMPNode::timer_clbk_1(void)
{
  count_1_++;
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Hello " << count_1_ << " from thread " << std::this_thread::get_id() << " (1)"
  );
}

void SMPNode::timer_clbk_2(void)
{
  count_2_++;
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Hello " << count_2_ << " from thread " << std::this_thread::get_id() << " (2)"
  );
}
